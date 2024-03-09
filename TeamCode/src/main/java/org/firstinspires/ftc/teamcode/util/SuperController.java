package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SuperController {

    private Telemetry telemetry;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private final double motorEncoderTicks = 1120;

    public static double maxPower = 0.45;
    public static double Kp_fightingGravity = 0.050; // 0.07
    public static double Ki_fightingGravity = 0; // 0.00065
    public static double Kd_fightingGravity = 0; // 0.0004
    public static double Kp_withGravity = 0.008; // 0.07
    public static double Ki_withGravity = 0; // 0.00065
    public static double Kd_withGravity = 0; // 0.0004
    public static double Kg = 0.102;
    public static double Kv = 0; // estimate is 1 / 2800, 0.00035714 -> 0.357143
    public static double Ka = 0;

    public SuperController() {
        initGainScheduling();
    }

    public void initGainScheduling() {

    }

    /**
     *
     * @param reference is the target position
     * @param state is the current position
     * @return power output of motor
     */
    public double calculatePID(double reference, double state) {
        double error = reference - state;
        integralSum += error * elapsedTime.seconds();
        double derivative = (error - lastError) / elapsedTime.seconds();
        lastError = error;

        elapsedTime.reset();

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

    /**
     *
     * @param targetPosition is the target position
     * @return power output of motor
     */
    public double positionalFeedforward(double targetPosition) {
        // convert target of 375 to 0 degrees
        double degrees = 375 - targetPosition;
        degrees = degrees / motorEncoderTicks * 360.0;

        return Kg * Math.sin( Math.toRadians(degrees) );
    }

    /**
     *
     * @param targetVelocity
     * @param targetAcceleration
     * @return
     */
    public double kvkaFeedforward(double targetVelocity, double targetAcceleration) {
        return (Kv / 1000 * targetVelocity) + (Ka / 1000 * targetAcceleration);
    }

}
