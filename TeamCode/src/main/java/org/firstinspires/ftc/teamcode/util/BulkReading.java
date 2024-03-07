package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

/**
 * Made to bulk read all sensors (not to be confused with the bulk read feature of lynx modules)
 */
public class BulkReading {

    private JVBoysSoccerRobot robot;
    private Telemetry telemetry;
    private HardwareMap hwMap;

    public static double pLauncherServo = 0;
    public static int pArmLeftMotor = 0;
    public static double vArmLeftMotor = 0;
    public static double pRigLeftServo = 0, pRigRightServo = 0;
    public static double pClawLeftServo = 0, pClawRightServo = 0;
    public static double pClawPivotLeftServo = 0, pClawPivotRightServo = 0;

    public BulkReading(JVBoysSoccerRobot robot, Telemetry telemetry, HardwareMap hwMap) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.hwMap = hwMap;
    }

    public void readAll() {
        pLauncherServo = robot.launcherFireServo.getPosition();

        pArmLeftMotor = robot.armLeftMotor.getCurrentPosition();
        vArmLeftMotor = robot.armLeftMotor.getVelocity();

        pRigLeftServo = robot.rigLeftServo.getPosition();
        pRigRightServo = robot.rigRightServo.getPosition();

        pClawLeftServo = robot.clawLeftServo.getPosition();
        pClawRightServo = robot.clawRightServo.getPosition();

        pClawPivotLeftServo = robot.clawPivotLeftServo.getPosition();
        pClawPivotRightServo = robot.clawPivotRightServo.getPosition();
    }

}
