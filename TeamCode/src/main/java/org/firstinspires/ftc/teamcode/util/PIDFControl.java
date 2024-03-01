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
    public static double Kp_fightingGravity = 0.057; // 0.07
    public static double Ki_fightingGravity = 0; // 0.00065
    public static double Kd_fightingGravity = 0; // 0.0004
    public static double Kp_withGravity = 0.008; // 0.07
    public static double Ki_withGravity = 0; // 0.00065
    public static double Kd_withGravity = 0; // 0.0004
    public static double maxPower = 0.45;
    public static double Kf = 0;
    private final double motorEncoderTicks = 1120;
    private final double ticksInDegrees = motorEncoderTicks / 360.0;
    private double lastError = 0;

    // K_pid values (gain scheduling) (gravity)
    private double Kp_at_10 = 0.05, Ki_at_10 = 0, Kd_at_10 = 0;
    private double Kp_at_50 = 0.055, Ki_at_50 = 0, Kd_at_50 = 0; // 0.00005
    private double Kp_at_110 = 0.06, Ki_at_110 = 0, Kd_at_110 = 0.00005; // 0.00015
    private double Kp_at_250 = 0.06, Ki_at_250 = 0, Kd_at_250 = 0.0003; // encoder tick = 375 + 110
    private double Kp_at_375 = 0, Ki_at_375 = 0, Kd_at_375 = 0; // encoder tick = 375 + 110

    // K values are less because with gravity
    private double Kp_at_10_lesser = 0.005, Ki_at_10_lesser = 0, Kd_at_10_lesser = 0;
    private double Kp_at_110_lesser = 0.01, Ki_at_110_lesser = 0, Kd_at_110_lesser = 0;
    private double Kp_at_300_lesser = 0.015, Ki_at_300_lesser = 0, Kd_at_300_lesser = 0;
    private double Kp_at_375_lesser = 0.02, Ki_at_375_lesser = 0, Kd_at_375_lesser = 0;

    // K_feedforward values (gain scheduling)
    private double Kf_at_110 = 0.003, Kf_at_top = 0, Kf_at_640 = -0.00045; // top = 375 encoder ticks
    private double Kf_at_500 = -0.0005, Kf_at_250 = 0.0014, Kf_at_0 = 0.0018;
    public InterpLUT KpCoefficients1 = new InterpLUT(),
            KiCoefficients1 = new InterpLUT(),
            KdCoefficients1 = new InterpLUT();
    public InterpLUT KpCoefficients2 = new InterpLUT(),
            KiCoefficients2 = new InterpLUT(),
            KdCoefficients2 = new InterpLUT();
    public InterpLUT KpCoefficientsLess1 = new InterpLUT(),
            KiCoefficientsLess1 = new InterpLUT(),
            KdCoefficientsLess1 = new InterpLUT();
    public InterpLUT KpCoefficientsLess2 = new InterpLUT(),
            KiCoefficientsLess2 = new InterpLUT(),
            KdCoefficientsLess2 = new InterpLUT();
    public InterpLUT KfCoefficients = new InterpLUT();

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

//        KpCoefficients1.add(10, Kp_at_10);
//        KiCoefficients1.add(10, Ki_at_10);
//        KdCoefficients1.add(10, Kd_at_10);
//
//        KpCoefficients1.add(50, Kp_at_50);
//        KiCoefficients1.add(50, Ki_at_50);
//        KdCoefficients1.add(50, Kd_at_50);
//
//        KpCoefficients1.add(110, Kp_at_110);
//        KiCoefficients1.add(110, Ki_at_110);
//        KdCoefficients1.add(110, Kd_at_110);
//
//        KpCoefficients1.add(250, Kp_at_250);
//        KiCoefficients1.add(250, Ki_at_250);
//        KdCoefficients1.add(250, Kd_at_250);
//
//        KpCoefficients1.add(375, Kp_at_375);
//        KiCoefficients1.add(375, Ki_at_375);
//        KdCoefficients1.add(375, Kd_at_375);
//
//        KpCoefficients2.add(375, Kp_at_375);
//        KiCoefficients2.add(375, Ki_at_375);
//        KdCoefficients2.add(375, Kd_at_375);
//
//        KpCoefficients2.add(500, Kp_at_250);
//        KiCoefficients2.add(500, Ki_at_250);
//        KdCoefficients2.add(500, Kd_at_250);
//
//        KpCoefficients2.add(640, Kp_at_110);
//        KiCoefficients2.add(640, Ki_at_110);
//        KdCoefficients2.add(640, Kd_at_110);

        KpCoefficientsLess1.add(10, Kp_at_10_lesser);
        KiCoefficientsLess1.add(10, Ki_at_10_lesser);
        KdCoefficientsLess1.add(10, Kd_at_10_lesser);

        KpCoefficientsLess1.add(110, Kp_at_110_lesser);
        KiCoefficientsLess1.add(110, Ki_at_110_lesser);
        KdCoefficientsLess1.add(110, Kd_at_110_lesser);

        KpCoefficientsLess1.add(300, Kp_at_300_lesser);
        KiCoefficientsLess1.add(300, Ki_at_300_lesser);
        KdCoefficientsLess1.add(300, Kd_at_300_lesser);

        KpCoefficientsLess1.add(375, Kp_at_375_lesser);
        KiCoefficientsLess1.add(375, Ki_at_375_lesser);
        KdCoefficientsLess1.add(375, Kd_at_375_lesser);

        KpCoefficientsLess2.add(375, Kp_at_375_lesser);
        KiCoefficientsLess2.add(375, Ki_at_375_lesser);
        KdCoefficientsLess2.add(375, Kd_at_375_lesser);

        KpCoefficientsLess2.add(450, Kp_at_300_lesser);
        KiCoefficientsLess2.add(450, Ki_at_300_lesser);
        KdCoefficientsLess2.add(450, Kd_at_300_lesser);

        KpCoefficientsLess2.add(640, Kp_at_110_lesser);
        KiCoefficientsLess2.add(640, Ki_at_110_lesser);
        KdCoefficientsLess2.add(640, Kd_at_110_lesser);

        KpCoefficientsLess2.add(740, Kp_at_10_lesser);
        KiCoefficientsLess2.add(740, Ki_at_10_lesser);
        KdCoefficientsLess2.add(740, Kd_at_10_lesser);

        KfCoefficients.add(0, Kf_at_0);
        KfCoefficients.add(110, Kf_at_110);
        KfCoefficients.add(250, Kf_at_250);
        KfCoefficients.add(375, Kf_at_top);
        KfCoefficients.add(640, Kf_at_500);

//        KpCoefficients1.createLUT();
//        KiCoefficients1.createLUT();
//        KdCoefficients1.createLUT();
//
//        KpCoefficients2.createLUT();
//        KiCoefficients2.createLUT();
//        KdCoefficients2.createLUT();

        KpCoefficientsLess1.createLUT();
        KiCoefficientsLess1.createLUT();
        KdCoefficientsLess1.createLUT();

        KpCoefficientsLess2.createLUT();
        KiCoefficientsLess2.createLUT();
        KdCoefficientsLess2.createLUT();

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

        boolean fightingGravity;
        double distance = reference - state;
        if (distance > 0) {
            fightingGravity = reference <= 375;
        }else {
            fightingGravity = !(reference <= 375);
        }

        double p = Kp_fightingGravity;
        double i = Ki_fightingGravity;
        double d = Kd_fightingGravity;

        if (!fightingGravity) {
            p = Kp_withGravity;
            i = Ki_withGravity;
            d = Kd_withGravity;
        }

        double output = (error * p) + (derivative * d) + (integralSum * i);

        if (output > maxPower) {
            output = maxPower;
        }

        return output;
    }

    public double calculatePID(double reference, double state, boolean isAngle, boolean gainScheduling) {
        boolean fightingGravity;
        double distance = reference - state;
        if (distance > 0) {
            fightingGravity = reference <= 375;
        }else {
            fightingGravity = !(reference <= 375);
        }

        double p = Kp_fightingGravity;
        double i = Ki_fightingGravity;
        double d = Kd_fightingGravity;

        if (!fightingGravity) {
            p = Kp_withGravity;
            i = Ki_withGravity;
            d = Kd_withGravity;
        }

        if (gainScheduling) {
//                if (state >= 375) {
//                    if (state >= 640) {
//                        p = KpCoefficients2.get(639);
//                        i = KiCoefficients2.get(639);
//                        d = KdCoefficients2.get(639);
//                    }else {
//                        p = KpCoefficients2.get(state);
//                        i = KiCoefficients2.get(state);
//                        d = KdCoefficients2.get(state);
//                    }
//                }else {
//                    if (state <= 10) {
//                        p = KpCoefficients1.get(11);
//                        i = KiCoefficients1.get(11);
//                        d = KdCoefficients1.get(11);
//                    }else {
//                        p = KpCoefficients1.get(state);
//                        i = KiCoefficients1.get(state);
//                        d = KdCoefficients1.get(state);
//                    }
//                }
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

        double output = (error * Kp_fightingGravity);

        if (output > maxPower) {
            output = maxPower;
        }

        return output;
    }

    public double calculateP(double reference, double state, boolean gainSchedule) {
        double error = reference - state;
        
        double kPValue;
        if (gainSchedule) {
//            kPValue = KpCoefficients1.get(state);
            kPValue = Kp_fightingGravity;
        }else {
            kPValue = Kp_fightingGravity;
        }

        double output = (error * kPValue);

        if (output > maxPower) {
            output = maxPower;
        }

        return output;
    }

    public double motionProfile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        /*
        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        */

        if (distance == 0 || max_acceleration == 0 || max_velocity == 0) {
            return 0;
        }
        if (distance < 0) {
            max_acceleration *= -1;
            max_velocity *= -1;
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
            target = 639;
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
