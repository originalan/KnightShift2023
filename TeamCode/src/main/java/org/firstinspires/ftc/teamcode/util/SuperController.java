package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SuperController {

    private Telemetry telemetry;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime elapsedTime2 = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private double lastVelocity = 0;
    private final double motorEncoderTicks = 1120;

    public static double maxPower = 0.45;
    public static double Kp_fightingGravity = 0.050; // 0.07
    public static double Ki_fightingGravity = 0; // 0.00065
    public static double Kd_fightingGravity = 0; // 0.0004
    public static double Kp_withGravity = 0.008; // 0.07
    public static double Ki_withGravity = 0; // 0.00065
    public static double Kd_withGravity = 0; // 0.0004
    public static double Kg = 0.102;
    private double K_v = 0; // estimate is 1 / 2800, 0.00035714 -> 0.357143
    private double K_a = 0;
    public static double FS_Kp = 0, FS_Kv = 0, FS_Ka = 0;
    public static double FS_Kp_g = 0;

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
        return (K_v / 1000 * targetVelocity) + (K_a / 1000 * targetAcceleration);
    }

    /**
     * Calculate state feedback for position and velocity of our system.
     * Math: Take the dot product of error and k values
     *          Error values are stored in a vector, same with k values, allowing us to take dot product
     */
    public double fullstateCalculate(double targetPosition, double targetVelocity, double robotPosition, double robotVelocity) {

        double positionError = targetPosition - robotPosition;
        double velocityError = targetVelocity - robotVelocity;
        double u = (positionError * FS_Kp) + (velocityError * FS_Kv);
        return u;

    }

    public double fullstateCalculate(double targetPosition, double targetVelocity, double targetAcceleration, double robotPosition, double robotVelocity) {

        double positionError = targetPosition - robotPosition;
        double velocityError = targetVelocity - robotVelocity;
        double r = (robotVelocity - lastVelocity) / elapsedTime2.seconds(); // current robot acceleration
        double accelerationError = targetAcceleration - r;

        lastVelocity = robotVelocity;
        elapsedTime2.reset();
        double u = (positionError * FS_Kp) + (velocityError * FS_Kv) + (accelerationError * FS_Ka);
        return u;

    }

    public double fullstateCalculateGravity(double targetPosition, double targetVelocity, double robotPosition, double robotVelocity) {

        double positionError = targetPosition - robotPosition;
        double velocityError = targetVelocity - robotVelocity;

        boolean fightingGravity;
        if (positionError > 0) {
            fightingGravity = targetPosition <= 375;
        }else {
            fightingGravity = !(targetPosition <= 375);
        }

        double p = FS_Kp;
        if (fightingGravity) {
            p = FS_Kp_g;
        }

        double u = (positionError * p) + (velocityError * FS_Kv);
        return u;

    }

    public double fullstateCalculateGravity(double targetPosition, double targetVelocity, double targetAcceleration, double robotPosition, double robotVelocity) {

        double positionError = targetPosition - robotPosition;
        double velocityError = targetVelocity - robotVelocity;
        double r = (robotVelocity - lastVelocity) / elapsedTime2.seconds(); // current robot acceleration
        double accelerationError = targetAcceleration - r;

        lastVelocity = robotVelocity;
        elapsedTime2.reset();

        boolean fightingGravity;
        if (positionError > 0) {
            fightingGravity = targetPosition <= 375;
        }else {
            fightingGravity = !(targetPosition <= 375);
        }

        double p = FS_Kp;
        if (fightingGravity) {
            p = FS_Kp_g;
        }

        double u = (positionError * p) + (velocityError * FS_Kv) + (accelerationError * FS_Ka);
        return u;

    }

}
