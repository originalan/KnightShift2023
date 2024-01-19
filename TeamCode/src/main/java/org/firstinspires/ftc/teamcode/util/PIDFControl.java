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
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static double motorEncoderTicks = 537.6;
    public static double ticksInDegrees = motorEncoderTicks / 360.0;
    public static double lastError = 0;

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

        KpCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (30.0 / 360.0 * 537.6 * 3), Kp_at_30);
        KiCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (30.0 / 360.0 * 537.6 * 3), Ki_at_30);
        KdCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (30.0 / 360.0 * 537.6 * 3), Kd_at_30);

        KpCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 537.6 * 3), Kp_at_150);
        KiCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 537.6 * 3), Ki_at_150);
        KdCoefficients.add(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 537.6 * 3), Kd_at_150);

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

        return output;
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
