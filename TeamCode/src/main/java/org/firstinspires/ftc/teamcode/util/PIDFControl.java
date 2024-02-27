package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An instance is made for every class that uses a PIDF loop
 */
@Config
public class PIDFControl {
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0;
    public static double Kp = 0; // 0.07
    public static double Ki = 0; // 0.00065
    public static double Kd = 0; // 0.0004
    public static double maxPower = 0.45;
    public static double Kf = 0;
    private final double motorEncoderTicks = 1120;
    private final double ticksInDegrees = motorEncoderTicks / 360.0;
    private double lastError = 0;

    // K_pid values (gain scheduling)
    private double Kp_at_10 = 0.02, Ki_at_10 = 0, Kd_at_10 = 0;
    private double Kp_at_50 = 0.055, Ki_at_50 = 0, Kd_at_50 = 0.00005;
    private double Kp_at_110 = 0.06, Ki_at_110 = 0, Kd_at_110 = 0.00015; // encoder tick = 110
    private double Kp_at_250 = 0.06, Ki_at_250 = 0, Kd_at_250 = 0.0003; // encoder tick = 375 + 110
    private double Kp_at_375 = 0, Ki_at_375 = 0, Kd_at_375 = 0; // encoder tick = 375 + 110

    // K_feedforward values (gain scheduling)
    private double Kf_at_110 = 0.003, Kf_at_top = 0, Kf_at_640 = -0.00045; // top = 375 encoder ticks
    private double Kf_at_500 = -0.0005, Kf_at_250 = 0.0014, Kf_at_0 = 0.0018;
    public InterpLUT KpCoefficients = new InterpLUT(),
    KiCoefficients = new InterpLUT(),
            KdCoefficients = new InterpLUT(),
            KfCoefficients = new InterpLUT();

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

        KpCoefficients.add(10, Kp_at_10);
        KiCoefficients.add(10, Ki_at_10);
        KdCoefficients.add(10, Kd_at_10);
        
        KpCoefficients.add(50, Kp_at_50);
        KiCoefficients.add(50, Ki_at_50);
        KdCoefficients.add(50, Kd_at_50);

        KpCoefficients.add(110, Kp_at_110);
        KiCoefficients.add(110, Ki_at_110);
        KdCoefficients.add(110, Kd_at_110);

        KpCoefficients.add(250, Kp_at_250);
        KiCoefficients.add(250, Ki_at_250);
        KdCoefficients.add(250, Kd_at_250);

        KpCoefficients.add(375, Kp_at_375);
        KiCoefficients.add(375, Ki_at_375);
        KdCoefficients.add(375, Kd_at_375);

        KpCoefficients.add(500, Kp_at_250);
        KiCoefficients.add(500, Ki_at_250);
        KdCoefficients.add(500, Kd_at_250);

        KpCoefficients.add(640, Kp_at_110);
        KiCoefficients.add(640, Ki_at_110);
        KdCoefficients.add(640, Kd_at_110);

        KfCoefficients.add(0, Kf_at_0);
        KfCoefficients.add(110, Kf_at_110);
        KfCoefficients.add(250, Kf_at_250);
        KfCoefficients.add(375, Kf_at_top);
        KfCoefficients.add(640, Kf_at_500);


        KpCoefficients.createLUT();
        KiCoefficients.createLUT();
        KdCoefficients.createLUT();
        KfCoefficients.createLUT();

    }

    /**
     * Calculates desired power of motor to retain a specific reference value
     * @param reference is the number of encoder ticks you want the motor to go (aka position)
     * @param state is the current encoder position of the motor
     * @param isAngle is true if you are controlling angles, false otherwise
     * @return power of desired motor to adjust
     */
    public double calculatePID(double reference, double state, boolean isAngle) {
        double error = isAngle ? angleWrap(reference - state) : (reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        return output;
    }

    public double calculatePID(double reference, double state, boolean isAngle, boolean gainScheduling) {
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

    public double calculateP(double reference, double state) {
        double error = reference - state;

        double output = (error * Kp);

        return output;
    }

    public double calculateP(double reference, double state, boolean gainSchedule) {
        double error = reference - state;
        
        double kPValue;
        if (gainSchedule) {
            kPValue = KpCoefficients.get(state);
        }else {
            kPValue = Kp;
        }

        double output = (error * kPValue);

        return output;
    }

    public double motionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        /*
        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        */

        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            return 0;
        }

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (Math.abs(acceleration_distance) > Math.abs(halfway_distance)) {
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

    public double calculateFeedforward(double target) {
        return (target / ticksInDegrees) * Kf;
    }

    public double calculateFeedforward(double target, boolean gainSchedule) {

        if (target <= 0) {
            target = 1;
        }
        if (target >= 640) {
            target = 640;
        }

        double kfValue;
        if (gainSchedule) {
            kfValue = KfCoefficients.get(target);
        }else {
            kfValue = Kf;
        }

        return (target / ticksInDegrees) * kfValue;
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
