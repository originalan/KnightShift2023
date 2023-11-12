package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An instance is made for every class that uses a PID loop
 */
public class PIDControl {

    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0;
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double Kf = 0; // For feedforward loop
    private double lastError = 0;

    // Used to control angle of robot
    private BNO055IMU imu; // Inertial measurement unit, built into expansion hub (has a gyro inside of it)

    private HardwareMap hwMap;

    public PIDControl() {

    }
    public PIDControl(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    /**
     * Calculates desired power of motor to retain a specific reference value
     * @param reference is the number of encoder ticks you want the motor to go (aka position)
     * @param state is the current encoder position of the motor
     * @param isAngle is true if you are controlling angles, false otherwise
     * @param isFeedForward is true if you want a feedforward loop added on for added accuracy
     *                      Note: feedforward loops do not work well for POSITION control (position includes angles)
     * @return power of desired motor to adjust
     */
    public double calculate(double reference, double state, boolean isAngle, boolean isFeedForward) {
        double error = isAngle ? angleWrap(reference - state) : (reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double feedForward = isFeedForward ? (reference * Kf) : 0;
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + feedForward;

        return output;
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
