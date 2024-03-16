package org.firstinspires.ftc.teamcode.settings;

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
    public static final String DRIVETRAIN_BACKLEFT_MOTOR_NAME = "BackLeftMotor";
    public static final String DRIVETRAIN_BACKRIGHT_MOTOR_NAME = "BackRightMotor";
    public static final String DRIVETRAIN_FRONTLEFT_MOTOR_NAME = "FrontLeftMotor";
    public static final String DRIVETRAIN_FRONTRIGHT_MOTOR_NAME = "FrontRightMotor";
    public static final boolean DRIVETRAIN_BACKLEFT_REVERSED = true;
    public static final boolean DRIVETRAIN_BACKRIGHT_REVERSED = false;
    public static final boolean DRIVETRAIN_FRONTLEFT_REVERSED = true;
    public static final boolean DRIVETRAIN_FRONTRIGHT_REVERSED = false;
    public static double DSENSOR_LEFT_OFFSET = 0.75;
    public static double DSENSOR_RIGHT_OFFSET = 0.5;

    public static final String DSENSOR_LEFT_NAME = "DSensorLeft";
    public static final String DSENSOR_RIGHT_NAME = "DSensorRight";

    /*
    =================CLAW SETTINGS==============
    */
    public static final String CLAW_RIGHT_SERVO_NAME = "ClawPieceLeft";
    public static final String CLAW_LEFT_SERVO_NAME = "ClawPieceRight"; // config names are flipped but variable names are accurate
    public static boolean CLAW_RIGHT_REVERSED = false;
    public static boolean CLAW_LEFT_REVERSED = true;
    public static final String CLAW_DSENSOR_LEFT = "ClawDLeft";
    public static final String CLAW_DSENSOR_RIGHT = "ClawDRight";
    public static double CLAW_LEFT_OFFSET = 0.3;
    public static double CLAW_RIGHT_OFFSET = 0.23;
    public static double CLAW_LEFT_THRESHOLD = 1.2;
    public static double CLAW_RIGHT_THRESHOLD = 1.2;

    /*
    ================= ARM SETTINGS==============
    */
    public static final String ARM_PIVOT_LEFT_SERVO_NAME = "ClawPivotLeftServo";
    public static boolean ARM_PIVOT_LEFT_SERVO_REVERSED = true;
    public static final String ARM_PIVOT_RIGHT_SERVO_NAME = "ClawPivotRightServo";
    public static boolean ARM_PIVOT_RIGHT_SERVO_REVERSED = false;
    public static final String ARM_MOTOR_LEFT_NAME = "ArmLeftMotor";
    public static final String ARM_MOTOR_RIGHT_NAME = "ArmRightMotor";
    public static boolean ARM_MOTOR_LEFT_REVERSED = true;
    public static boolean ARM_MOTOR_RIGHT_REVERSED = false;

    /*
    =================RIGGING SETTINGS==============
    */
    public static final String RIGGING_LEFT_MOTOR_NAME = "LeftRiggingMotor";
    public static final String RIGGING_RIGHT_MOTOR_NAME = "RightRiggingMotor";
    public static final String RIGGING_LEFT_SERVO_NAME = "RightRiggingServo";
    public static final String RIGGING_RIGHT_SERVO_NAME = "LeftRiggingServo";
    public static final boolean RIGGING_LEFT_MOTOR_REVERSED = true;
    public static final boolean RIGGING_RIGHT_MOTOR_REVERSED = false;
    public static final boolean RIGGING_LEFT_SERVO_REVERSED = true;
    public static final boolean RIGGING_RIGHT_SERVO_REVERSED = false;
    public static double RIGGING_MOTOR_SPEED = 1;
    public static double RIGGING_LEFT_REST = 0.04; // LEFT AND RIGHT SERVOS ARE SWITCHED (by accident)
    public static double RIGGING_RIGHT_REST = 0;
    public static double RIGGING_LEFT_TOP = 0.97;
    public static double RIGGING_RIGHT_TOP = 0.89;
    public static double RIGGING_RESET_MOTOR_SPEED = 0.2;

    /*
    =================AIRPLANE LAUNCHER SETTINGS==============
    */
    public static final String LAUNCHER_FIRE_SERVO_NAME = "AirplaneLauncherFireServo";
    public static final String LAUNCHER_NAME = "AirplaneServo";
    public static boolean LAUNCHER_FIRE_SERVO_REVERSED = false;
    public static final String LAUNCHER_ADJUST_SERVO_NAME = "AirplaneLauncherAdjustServo";
    public static boolean LAUNCHER_ADJUST_SERVO_REVERSED = false;
    public static double LAUNCHER_FIRE_POSITION_REST = 0.6;
    public static double LAUNCHER_FIRE_POSITION_FIRE = 0.8;
    public static double LAUNCHER_ADJUST_POSITION_UNCLAMPED = 0.9;
    public static double LAUNCHER_ADJUST_POSITION_CLAMPED = 0.9;

    /*
    =================MISC SETTINGS==============
    */
    public static final String CAMERA_NAME = "Webcam 1";
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

}
