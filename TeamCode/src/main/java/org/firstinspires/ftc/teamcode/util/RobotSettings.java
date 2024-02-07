package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * RobotSettings is a class that stores all hardware settings and constants.
 * Can be changed live in FTC Dashboard
 */
@Config
public class RobotSettings {

    /*
    =================DRIVETRAIN SETTINGS==============
    */
    public static String DRIVETRAIN_BACKLEFT_MOTOR_NAME = "BackLeftMotor";
    public static String DRIVETRAIN_BACKRIGHT_MOTOR_NAME = "BackRightMotor";
    public static String DRIVETRAIN_FRONTLEFT_MOTOR_NAME = "FrontLeftMotor";
    public static String DRIVETRAIN_FRONTRIGHT_MOTOR_NAME = "FrontRightMotor";
    public static boolean DRIVETRAIN_BACKLEFT_REVERSED = true;
    public static boolean DRIVETRAIN_BACKRIGHT_REVERSED = false;
    public static boolean DRIVETRAIN_FRONTLEFT_REVERSED = true;
    public static boolean DRIVETRAIN_FRONTRIGHT_REVERSED = false;

    /*
    =================CLAW SETTINGS==============
    */
    public static String CLAW_LEFT_SERVO_NAME = "ClawPieceLeftServo";
    public static String CLAW_RIGHT_SERVO_NAME = "ClawPieceRightServo";
    public static boolean CLAW_LEFT_REVERSED = false;
    public static boolean CLAW_RIGHT_REVERSED = true;
    public static double CLAW_LEFT_CLOSE = 0.265;
    public static double CLAW_LEFT_OPEN = 0;
    public static double CLAW_RIGHT_CLOSE = 0.265;
    public static double CLAW_RIGHT_OPEN = 0;

    /*
    ================= ARM SETTINGS==============
    */
    public static String ARM_PIVOT_SERVO_NAME = "ClawPivotServo";
    public static boolean ARM_PIVOT_REVERSED = false;
    public static double ARM_PIVOT_REST = 0;
    public static String ARM_MOTOR_LEFT_NAME = "ArmMotorLeft";
    public static String ARM_MOTOR_RIGHT_NAME = "ArmMotorRight";
    public static boolean ARM_MOTOR_LEFT_REVERSED = false;
    public static boolean ARM_MOTOR_RIGHT_REVERSED = true;

    /*
    =================RIGGING SETTINGS==============
    */
    public static String RIGGING_LEFT_MOTOR_NAME = "LeftRiggingMotor";
    public static String RIGGING_RIGHT_MOTOR_NAME = "RightRiggingMotor";
    public static String RIGGING_LEFT_SERVO_NAME = "RightRiggingServo";
    public static String RIGGING_RIGHT_SERVO_NAME = "LeftRiggingServo";
    public static boolean RIGGING_LEFT_MOTOR_REVERSED = true;
    public static boolean RIGGING_RIGHT_MOTOR_REVERSED = false;
    public static boolean RIGGING_LEFT_SERVO_REVERSED = true;
    public static boolean RIGGING_RIGHT_SERVO_REVERSED = false;
    public static double RIGGING_MOTOR_SPEED = 0.6;
    public static double RIGGING_LEFT_REST = 0;
    public static double RIGGING_RIGHT_REST = 0;
    public static double RIGGING_LEFT_TOP = 1;
    public static double RIGGING_RIGHT_TOP = 1;
    public static double RIGGING_RESET_MOTOR_SPEED = 0.2;

    /*
    =================AIRPLANE LAUNCHER SETTINGS==============
    */
    public static String LAUNCHER_FIRE_SERVO_NAME = "AirplaneLauncherFireServo";
    public static boolean LAUNCHER_FIRE_SERVO_REVERSED = false;
    public static String LAUNCHER_ADJUST_SERVO_NAME = "AirplaneLauncherAdjustServo";
    public static boolean LAUNCHER_ADJUST_SERVO_REVERSED = false;
    public static double LAUNCHER_FIRE_POSITION_REST = 0.8;
    public static double LAUNCHER_FIRE_POSITION_FIRE = 0.2;
    public static double LAUNCHER_ADJUST_POSITION_UNCLAMPED = 0.9;
    public static double LAUNCHER_ADJUST_POSITION_CLAMPED = 0.9;

    /*
    =================MISC SETTINGS==============
    */
    public static String CAMERA_NAME = "Webcam 1";
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

}
