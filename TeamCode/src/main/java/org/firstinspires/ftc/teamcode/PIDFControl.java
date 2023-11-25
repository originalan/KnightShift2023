package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An instance is made for every class that uses a PID loop
 */
@Config
public class PIDFControl {

    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static double ticksInDegrees = 1440 / 180.0;
    public static double lastError = 0;

    // Used to control angle of robot
    private BNO055IMU imu; // Inertial measurement unit, built into expansion hub (has a gyro inside of it)

    private HardwareMap hwMap;

    public PIDFControl() {

    }
    public PIDFControl(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
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

    public double feedForwardCalculate(double state) {
        double ffPower = Math.cos(Math.toRadians(state / ticksInDegrees)) + Kf;
        return ffPower;
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