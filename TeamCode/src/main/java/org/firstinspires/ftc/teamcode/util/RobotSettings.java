package org.firstinspires.ftc.teamcode.util;

public class RobotSettings {

    /*
    =================DRIVETRAIN SETTINGS==============
    */
    public static final String DRIVETRAIN_BACKLEFT_MOTOR_NAME = "backLeft";
    public static final String DRIVETRAIN_BACKRIGHT_MOTOR_NAME = "backRight";
    public static final String DRIVETRAIN_FRONTLEFT_MOTOR_NAME = "frontLeft";
    public static final String DRIVETRAIN_FRONTRIGHT_MOTOR_NAME = "frontRight";
    public static final double DRIVETRAIN_kV = -1;
    public static final double DRIVETRAIN_kA = -1;
    public static final double DRIVETRAIN_kStatic = -1;
    public static final boolean DRIVETRAIN_USE_ENCODERS = false;
    public static final double DRIVETRAIN_LENGTH = -1;
    public static final double DRIVETRAIN_WIDTH = -1;

    /*
    =================INTAKE SETTINGS==============
    */
    public static final String INTAKE_MOTOR_NAME = "Intake";
    public static final boolean INTAKE_MOTOR_REVERSED = true;
    public static final double INTAKE_DEFAULT_MOTOR_SPEED = 1.0;
    public static final double INTAKE_CURRENT_THRESHOLD = 10000;

    /*
    =================OUTTAKE SETTINGS==============
    */
    public static final String OUTTAKE_MOTOR_NAME = "LinearSlide";
    public static final boolean OUTTAKE_MOTOR_REVERSED = false;
    public static final String PURPLE_PIXEL_SERVO_NAME = "PurplePixel";
    public static final boolean PURPLE_PIXEL_SERVO_REVERSED = false;

    /*
    =================RIGGING SETTINGS==============
    */
    public static final String RIGGING_LEFT_MOTOR_NAME = "LeftRigging";
    public static final String RIGGING_RIGHT_MOTOR_NAME = "RightRigging";
    public static final String RIGGING_LEFT_SERVO_NAME = "RightRiggingServo";
    public static final String RIGGING_RIGHT_SERVO_NAME = "LeftRiggingServo";
    public static final boolean RIGGING_LEFT_MOTOR_REVERSED = false;
    public static final boolean RIGGING_RIGHT_MOTOR_REVERSED = false;
    public static final boolean RIGGING_LEFT_SERVO_REVERSED = false;
    public static final boolean RIGGING_RIGHT_SERVO_REVERSED = false;
    public static final double ROBOT_FRONT_LENGTH = 16.75;
    public static final double ROBOT_SIDE_LENGTH = 18.00,
                                ROBOT_LOCATION_1 = 9.875, // Location of the purple pixel servo box from the LEFT side of the robot
                                ROBOT_LOCATION_2 = 10.75; // Location of yellow pixel arm relative to the LEFT side of the robot
    /*
    =================AIRPLANE LAUNCHER SETTINGS==============
    */
    public static final String LAUNCHER_SERVO_NAME = "AirplaneLauncher";
    public static final boolean LAUNCHER_SERVO_REVERSED = false;
    public static final double LAUNCHER_POSITION_IN = 0;
    public static final double LAUNCHER_POSITION_OUT = 1;

    /*
    =================SENSOR SETTINGS==============
    */
    public static final String CAMERA_NAME = "Webcam 1";

}
