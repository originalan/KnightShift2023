package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PIDControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Robot super system - JV BOYS SOCCER TEAM
 */
public class JVBoysSoccerRobot {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private List<LynxModule> allHubs;
    public PIDControl pid;

    // Subsystems
    public Drivetrain drivetrain;
    public Intake intake;
    public LinearSlide slide;
    public Rigging rig;
    public AirplaneLauncher launcher;
    public PurplePixel purplePixel;

    // Hardware
    public BNO055IMU imu;
    public IMU imu2;
    public DcMotorEx backRight, backLeft, frontRight, frontLeft;
    public DcMotorEx intakeMotor;
    public Servo airplaneLauncherServo;
    public DcMotorEx linearSlideMotor;
    public Servo linearSlideServo;
    public Servo leftRigServo;
    public Servo rightRigServo;
    public DcMotorEx leftRigMotor;
    public DcMotorEx rightRigMotor;
    public Servo purplePixelServo;

    // Integrated Heading for IMU
    private double previousHeading = 0;
    private double integratedHeading = 0;

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
        pid = new PIDControl();

        // Configuring Hubs to auto mode for bulk reads
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        initIMU();
        initHardware();
        drivetrain = new Drivetrain(hwMap, telemetry, this);
        intake = new Intake(hwMap, telemetry, this);
        slide = new LinearSlide(hwMap, telemetry, this);
        rig = new Rigging(hwMap, telemetry, this);
        launcher = new AirplaneLauncher(hwMap, telemetry, this);
        purplePixel = new PurplePixel(hwMap, telemetry, this);

        subsystems = Arrays.asList(drivetrain, intake, slide, rig, launcher);
    }

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry, AllianceType allianceType) {
        this(hwMap, telemetry);
        this.allianceType = allianceType;
    }

    public void addTelemetry() {
        for (Subsystem s : subsystems) {
            s.addTelemetry();
        }
        telemetry.addLine("Misc.");
        telemetry.addData("   IMU Absolute Angle Rotation", getIntegratedHeading());
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
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        imu2 = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters1 = new IMU.Parameters(new RevHubOrientationOnRobot(
                RobotSettings.LOGO_FACING_DIR, RobotSettings.USB_FACING_DIR));
        imu2.initialize(parameters1);
    }

    public void initHardware() {
        // Airplane Launcher Subsystem
        airplaneLauncherServo = hwMap.servo.get(RobotSettings.LAUNCHER_SERVO_NAME);
        airplaneLauncherServo.setDirection(RobotSettings.LAUNCHER_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        airplaneLauncherServo.setPosition(0);

        // Drivetrain Subsystem
        backLeft = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_BACKLEFT_MOTOR_NAME);
        backRight = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_BACKRIGHT_MOTOR_NAME);
        frontLeft = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_FRONTLEFT_MOTOR_NAME);
        frontRight = hwMap.get(DcMotorEx.class, RobotSettings.DRIVETRAIN_FRONTRIGHT_MOTOR_NAME);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

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

        // Intake Subsystem
        intakeMotor = hwMap.get(DcMotorEx.class, RobotSettings.INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(RobotSettings.INTAKE_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        // Purple Pixel Subsystem
        purplePixelServo = hwMap.servo.get(RobotSettings.PURPLE_PIXEL_SERVO_NAME);
        purplePixelServo.setDirection(RobotSettings.PURPLE_PIXEL_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        // Linear Slide Subsystem
        linearSlideMotor = hwMap.get(DcMotorEx.class, RobotSettings.OUTTAKE_MOTOR_NAME);
        linearSlideMotor.setDirection(RobotSettings.OUTTAKE_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        linearSlideServo = hwMap.servo.get(RobotSettings.OUTTAKE_SERVO_CLAW_NAME);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideServo.setDirection(RobotSettings.OUTTAKE_SERVO_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        // Rigging Subsystem
        leftRigServo = hwMap.servo.get(RobotSettings.RIGGING_LEFT_SERVO_NAME);
        rightRigServo = hwMap.servo.get(RobotSettings.RIGGING_RIGHT_SERVO_NAME);
        leftRigServo.setDirection(RobotSettings.RIGGING_LEFT_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightRigServo.setDirection(RobotSettings.RIGGING_RIGHT_SERVO_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        leftRigMotor = hwMap.get(DcMotorEx.class, RobotSettings.RIGGING_LEFT_MOTOR_NAME);
        rightRigMotor = hwMap.get(DcMotorEx.class, RobotSettings.RIGGING_RIGHT_MOTOR_NAME);
        leftRigMotor.setDirection(RobotSettings.RIGGING_LEFT_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rightRigMotor.setDirection(RobotSettings.RIGGING_RIGHT_MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        leftRigMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRigMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Records the absolute angle of the imu compared to when it first started (-infinity, infinity)
     * @return
     */
    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }
        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }

    public void resetIMUAngle() {
        integratedHeading = 0;
        previousHeading = 0;
    }

//    public String formatDegrees(double degrees) {
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }

}
