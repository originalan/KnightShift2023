package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

/**
 * An instance is made for every class that uses a PIDF loop
 */
@Config
public class PIDFControl {
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0;
    public static double Kp = 0.07;
    public static double Ki = 0.00065;
    public static double Kd = 0.0004;
    public static double maxPower = 0.45;
    private double Kf = 0;
    private final double motorEncoderTicks = 1120;
    private final double ticksInDegrees = motorEncoderTicks / 360.0;
    private double lastError = 0;

    // Gain scheduling (if we have time)
    public static double Kp_at_150 = 0, Ki_at_150 = 0, Kd_at_150 = 0; // encoder tick = 672
    public static double Kp_at_30 = 0, Ki_at_30 = 0, Kd_at_30 = 0; // encoder tick = 134.4
    private InterpLUT KpCoefficients = new InterpLUT(),
            KiCoefficients = new InterpLUT(),
            KdCoefficients = new InterpLUT();

    private HardwareMap hwMap;

    public PIDFControl() {
        initGainScheduling();
    }
    public PIDFControl(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        initGainScheduling();
    }

    public void initGainScheduling() {

        KpCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (30.0 / 360.0 * 1120), Kp_at_30);
        KiCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (30.0 / 360.0 * 1120), Ki_at_30);
        KdCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (30.0 / 360.0 * 1120), Kd_at_30);

        KpCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 1120), Kp_at_150);
        KiCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 1120), Ki_at_150);
        KdCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 1120), Kd_at_150);

        KpCoefficients.createLUT();
        KiCoefficients.createLUT();
        KdCoefficients.createLUT();

    }

    /**
     * Calculates desired power of motor to retain a specific reference value
     * @param reference is the number of encoder ticks you want the motor to go (aka position)
     * @param state is the current encoder position of the motor
     * @param isAngle is true if you are controlling angles, false otherwise
     * @return power of desired motor to adjust
     */
    public double calculate(double reference, double state, boolean isAngle) {
        double error = isAngle ? angleWrap(reference - state) : (reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        return output;
    }

    public double calculate(double reference, double state, boolean isAngle, boolean gainScheduling) {
        double p = Kp;
        double i = Ki;
        double d = Kd;

        if (gainScheduling) {
            p = KpCoefficients.get(state);
            i = KiCoefficients.get(state);
            d = KdCoefficients.get(state);
        }

        double error = isAngle ? angleWrap(reference - state) : (reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * p) + (derivative * d) + (integralSum * i);

        if (output > maxPower) {
            output = maxPower;
        }

        return output;
    }

    public double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        /*
        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        */

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }

    public double feedForwardCalculate(double state) {
        return Math.cos(Math.toRadians(state / ticksInDegrees)) * Kf;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -1 * Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

}
