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
    public static boolean DRIVETRAIN_USE_ENCODERS = false;

    /*
    =================INTAKE SETTINGS==============
    */
    public static String INTAKE_MOTOR_NAME = "IntakeMotor";
    public static boolean INTAKE_MOTOR_REVERSED = false;
    public static double INTAKE_FORWARD_MOTOR_SPEED = 0.75;
    public static double INTAKE_REVERSE_MOTOR_SPEED = -0.25;

    /*
    =================DELIVERY ARM SETTINGS==============
    */
    public static String OUTTAKE_MOTOR_NAME = "ArmMotor";
    public static boolean OUTTAKE_MOTOR_REVERSED = false;
    public static double ARM_TOP_POSITION_DEGREES = 150.0;
    public static double ARM_MOTOR_POWER = 1;
    public static int ARM_FLIP_POSITION = 550;
    public static double ARM_BOTTOM_POSITION_DEGREES = 0;
    public static int ARM_ENCODER_TOP = (int)(ARM_TOP_POSITION_DEGREES / 360.0 * 537.6);


    /*
    =================RIGGING SETTINGS==============
    */
    public static String RIGGING_LEFT_MOTOR_NAME = "LeftRiggingMotor";
    public static String RIGGING_RIGHT_MOTOR_NAME = "RightRiggingMotor";
    public static String RIGGING_LEFT_SERVO_NAME = "RightRiggingServo";
    public static String RIGGING_RIGHT_SERVO_NAME = "LeftRiggingServo";
    public static boolean RIGGING_LEFT_MOTOR_REVERSED = false;
    public static boolean RIGGING_RIGHT_MOTOR_REVERSED = false;
    public static boolean RIGGING_LEFT_SERVO_REVERSED = false;
    public static boolean RIGGING_RIGHT_SERVO_REVERSED = false;
    public static double RIGGING_MOTOR_SPEED = 0.6;
    public static double RIGGING_LEFT_REST = 0.975; // left and right servos are switched irl
    public static double RIGGING_RIGHT_REST = 0.0;
    public static double RIGGING_MOVE_SERVO = -0.25; // gear ratio is 2:1 so this moves it 90 degrees

    public static double RIGGING_RESET_MOTOR_SPEED = 0.2;

    /*
    =================AUTONOMOUS MEASUREMENTS==============
    */
    public static double ROBOT_FRONT_LENGTH = 16.75;
    public static double ROBOT_SIDE_LENGTH = 18.00,
                                ROBOT_LOCATION_1 = 9.875, // Location of the purple pixel servo box from the LEFT side of the robot
                                ROBOT_LOCATION_2 = 10.75, // Location of yellow pixel arm relative to the LEFT side of the robot
                                ROBOT_LOCATOIN_3 = 0.75, // length of little edges of tiles
                                ROBOT_LOCATION_4 = 23.5, // length of tile without one of the little edges
                                ROBOT_LOCATION_5 = 5.25, // length of left side of board to the first divet for the pixel
                                ROBOT_LOCATION_6 = 3.00; // distance between divets in the board inches
    /*
    =================AIRPLANE LAUNCHER SETTINGS==============
    */
    public static String LAUNCHER_SERVO_NAME = "AirplaneLauncherFireServo";
    public static boolean LAUNCHER_SERVO_REVERSED = false;
    public static String LAUNCHER_SERVO_NAME_2 = "AirplaneLauncherAdjustServo";
    public static boolean LAUNCHER_SERVO_REVERSED_2 = false;
    public static double LAUNCHER_FIRE_POSITION_REST = 0.2;
    public static double LAUNCHER_FIRE_POSITION_FIRE = 0.8;
    public static double LAUNCHER_ADJUST_POSITION_UNCLAMPED = 0.2;
    public static double LAUNCHER_ADJUST_POSITION_CLAMPED = 0;

    /*
    =================MISC SETTINGS==============
    */
    public static String CAMERA_NAME = "Webcam 1";
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public static double AUTO_PURPLE_PIXEL_RELEASE = 8.0;

}
