package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotSettings {

    /*
    =================DRIVETRAIN SETTINGS==============
    */
    public static String DRIVETRAIN_BACKLEFT_MOTOR_NAME = "backLeft";
    public static String DRIVETRAIN_BACKRIGHT_MOTOR_NAME = "backRight";
    public static String DRIVETRAIN_FRONTLEFT_MOTOR_NAME = "frontLeft";
    public static String DRIVETRAIN_FRONTRIGHT_MOTOR_NAME = "frontRight";
    public static boolean DRIVETRAIN_USE_ENCODERS = false;

    /*
    =================INTAKE SETTINGS==============
    */
    public static String INTAKE_MOTOR_NAME = "Intake";
    public static boolean INTAKE_MOTOR_REVERSED = true;
    public static double INTAKE_DEFAULT_MOTOR_SPEED = 0.4;
    public static double INTAKE_CURRENT_THRESHOLD = 10000;

    /*
    =================OUTTAKE SETTINGS==============
    */
    public static String OUTTAKE_MOTOR_NAME = "LinearSlide";
    public static boolean OUTTAKE_MOTOR_REVERSED = true;
    public static int OUTTAKE_MOTOR_ENCODER_POSITION = (int)((80.0 / 360.0) * 1440.0);
    public static double OUTTAKE_MOTOR_POWER = 0.25;
    public static String OUTTAKE_SERVO_CLAW_NAME = "YellowPixelServo";
    public static boolean OUTTAKE_SERVO_CLAW_REVERSED = false;
    public static double OUTTAKE_SERVO_CLAW_STARTING_POSITION = 0.5;
    public static double OUTTAKE_SERVO_CLAW_HOLDING_POSITION = 0.6;
    public static double OUTTAKE_SERVO_CLAW_RELEASE_POSITION = 0.4;

    // Purple Pixel Stuff
    public static String PURPLE_PIXEL_SERVO_NAME = "PurplePixel";
    public static boolean PURPLE_PIXEL_SERVO_REVERSED = false;
    public static double PURPLE_PIXEL_SERVO_STARTING_POSITION = 1.0;
    public static double PURPLE_PIXEL_SERVO_ENDING_POSITION = 1.0 - (22.0 / 180.0);

    /*
    =================RIGGING SETTINGS==============
    */
    public static String RIGGING_LEFT_MOTOR_NAME = "LeftRigging";
    public static String RIGGING_RIGHT_MOTOR_NAME = "RightRigging";
    public static String RIGGING_LEFT_SERVO_NAME = "RightRiggingServo";
    public static String RIGGING_RIGHT_SERVO_NAME = "LeftRiggingServo";
    public static boolean RIGGING_LEFT_MOTOR_REVERSED = false;
    public static boolean RIGGING_RIGHT_MOTOR_REVERSED = false;
    public static boolean RIGGING_LEFT_SERVO_REVERSED = false;
    public static boolean RIGGING_RIGHT_SERVO_REVERSED = false;
    public static double RIGGING_MOTOR_SPEED = 0.4;

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
    public static String LAUNCHER_SERVO_NAME = "AirplaneLauncher";
    public static boolean LAUNCHER_SERVO_REVERSED = false;
    public static double LAUNCHER_POSITION_IN = 0;
    public static double LAUNCHER_POSITION_OUT = 1;

    /*
    =================SENSOR SETTINGS==============
    */
    public static String CAMERA_NAME = "Webcam 1";

}
