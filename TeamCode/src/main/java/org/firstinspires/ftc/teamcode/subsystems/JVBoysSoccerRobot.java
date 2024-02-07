package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

import java.util.Arrays;
import java.util.List;

/**
 * JVBoysSoccerRobot is the robot base superclass.
 * All hardware and subsystems are initialized here.
 * GO JV BOYS SOCCER TEAM!
 */
public class JVBoysSoccerRobot {

    public static int initialArmPosition;
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private List<LynxModule> allHubs;
    public PIDFControl pid;

    // Subsystems
    public Drivetrain drivetrainSubsystem;
    public Claw clawSubsystem;
    public Arm armSubsystem;
    public Rigging riggingSubsystem;
    public AirplaneLauncher launcherSubsystem;

    // Hardware
    public IMU imu2;
    public DcMotorEx backRight, backLeft, frontRight, frontLeft;
    
    public Servo launcherFireServo;
    public Servo launcherAdjustServo;
    
    public DcMotorEx armLeftMotor;
    public DcMotorEx armRightMotor;
    
    public Servo rigLeftServo;
    public Servo rigRightServo;
    public DcMotorEx rigLeftMotor;
    public DcMotorEx rigRightMotor;
    
    public Servo clawLeftServo;
    public Servo clawRightServo;
    public Servo clawPivotServo;

    // Alliance Type
    public enum AllianceType {
        RED,
        BLUE
    }
    public AllianceType allianceType;

    private List<Subsystem> subsystems;

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        pid = new PIDFControl();

        // Configuring Hubs to auto mode for bulk reads
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        initIMU();
        initHardware();
        drivetrainSubsystem = new Drivetrain(hwMap, telemetry, this);
        clawSubsystem = new Claw(hwMap, telemetry, this);
        armSubsystem = new Arm(hwMap, telemetry, this);
        riggingSubsystem = new Rigging(hwMap, telemetry, this);
        launcherSubsystem = new AirplaneLauncher(hwMap, telemetry, this);

        subsystems = Arrays.asList(drivetrainSubsystem, clawSubsystem, armSubsystem, riggingSubsystem, launcherSubsystem);
    }

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry, AllianceType allianceType) {
        this(hwMap, telemetry);
        this.allianceType = allianceType;
    }

    public void addTelemetry() {
        for (Subsystem s : subsystems) {
            s.addTelemetry();
        }
    }

    public void update() {
        for (Subsystem s : subsystems) {
            s.update();
        }
    }

    public void stop() {
        for (Subsystem s : subsystems) {
            s.stop();
        }
    }

    public void initIMU() {
        imu2 = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters1 = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotSettings.LOGO_FACING_DIR, RobotSettings.USB_FACING_DIR));
        imu2.initialize(parameters1);
    }

    public void initHardware() {
        // Airplane Launcher Subsystem
        initLauncherHardware();
        
        // Drivetrain Subsystem
        initDrivetrainHardware();

        // Claw Subsystem
        initClawHardware();

        // Arm Subsystem
        initArmHardware();

        // Rigging Subsystem
        initRiggingHardware();
    }
    
    public void initLauncherHardware() {
        launcherFireServo = hwMap.servo.get(RobotSettings.LAUNCHER_FIRE_SERVO_NAME);
        launcherFireServo.setDirection(RobotSettings.LAUNCHER_FIRE_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        launcherAdjustServo = hwMap.servo.get(RobotSettings.LAUNCHER_ADJUST_SERVO_NAME);
        launcherAdjustServo.setDirection(RobotSettings.LAUNCHER_ADJUST_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }
    
    public void initDrivetrainHardware() {
        backLeft = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_BACKLEFT_MOTOR_NAME);
        backRight = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_BACKRIGHT_MOTOR_NAME);
        frontLeft = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_FRONTLEFT_MOTOR_NAME);
        frontRight = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_FRONTRIGHT_MOTOR_NAME);

        backLeft.setDirection(RobotSettings.DRIVETRAIN_BACKLEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backRight.setDirection(RobotSettings.DRIVETRAIN_BACKRIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frontLeft.setDirection(RobotSettings.DRIVETRAIN_FRONTLEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frontRight.setDirection(RobotSettings.DRIVETRAIN_FRONTRIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        // This DOES NOT disable the tick counts
        // What it DOES do is disable the built in feedback loop
        // because external feedback such as our PID controller is generally preferred (and way faster)
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void initClawHardware() {
        clawLeftServo = hwMap.servo.get(RobotSettings.CLAW_LEFT_SERVO_NAME);
        clawRightServo = hwMap.servo.get(RobotSettings.CLAW_RIGHT_SERVO_NAME);

        clawLeftServo.setDirection(RobotSettings.CLAW_LEFT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        clawRightServo.setDirection(RobotSettings.CLAW_RIGHT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }
    
    public void initArmHardware() {
        armLeftMotor = hwMap.get(DcMotorEx.class, RobotSettings.ARM_MOTOR_LEFT_NAME);
        armLeftMotor.setDirection(RobotSettings.ARM_MOTOR_LEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRightMotor = hwMap.get(DcMotorEx.class, RobotSettings.ARM_MOTOR_RIGHT_NAME);
        armRightMotor.setDirection(RobotSettings.ARM_MOTOR_RIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clawPivotServo = hwMap.servo.get(RobotSettings.ARM_PIVOT_SERVO_NAME);
        clawPivotServo.setDirection(RobotSettings.ARM_PIVOT_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }
    
    public void initRiggingHardware() {
        rigLeftServo = hwMap.servo.get(RobotSettings.RIGGING_LEFT_SERVO_NAME);
        rigRightServo = hwMap.servo.get(RobotSettings.RIGGING_RIGHT_SERVO_NAME);
        rigLeftServo.setDirection(RobotSettings.RIGGING_LEFT_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rigRightServo.setDirection(RobotSettings.RIGGING_RIGHT_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        rigLeftMotor = hwMap.get(DcMotorEx.class, RobotSettings.RIGGING_LEFT_MOTOR_NAME);
        rigRightMotor = hwMap.get(DcMotorEx.class, RobotSettings.RIGGING_RIGHT_MOTOR_NAME);
        rigLeftMotor.setDirection(RobotSettings.RIGGING_LEFT_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rigRightMotor.setDirection(RobotSettings.RIGGING_RIGHT_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rigLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rigRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
