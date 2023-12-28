package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 *  An instance is made for every class that uses Full State Feedback
 *  Similar to PIDF loop
 *  It is essentially a proportional derivative controller but you can control multiple states compared to PIDF only controlling one
 *  Unlike PIDF, the target values such as velocity do not necessarily have to be zero, allowing for more robust control and possible implementations of motion profiles
 */
@Config
public class FullStateFeedback {

    private Telemetry telemetry;
    private HardwareMap hwMap;

    public static double kP = 0; // k for position state
    public static double kV = 0; // k for velocity state

    public FullStateFeedback(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    public FullStateFeedback(HardwareMap hwMap, Telemetry telemetry, double kP, double kV) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.kP = kP;
        this.kV = kV;
    }

    public void setCoefficients(double kP, double kV) {
        this.kP = kP;
        this.kV = kV;
    }

    /**
     * Calculate state feedback for position and velocity of our system.
     * Math: Take the dot product of error and k values
     *          Error values are stored in a vector, same with k values, allowing us to take dot product
     */
    public double calculate(double targetPosition, double targetVelocity, double robotPosition, double robotVelocity) {

        double positionError = targetPosition - robotPosition;
        double velocityError = targetVelocity - robotVelocity;
        double u = (positionError * kP) + (velocityError * kV);
        return u;

    }

}
