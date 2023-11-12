package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDControl;

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

    // Hardware
    public BNO055IMU imu;
    public DcMotorEx backRight, backLeft, frontRight, frontLeft;
    public DcMotorEx intakeMotor;
    public Servo airplaneLauncherServo;
    public DcMotorEx linearSlideMotor;
    public Servo leftRigServo;
    public Servo rightRigServo;
    public DcMotorEx leftRigMotor;
    public DcMotorEx rightRigMotor;

    private List<Subsystem> subsystems;

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        pid = new PIDControl();

        // Configuring Hubs to auto mode for bulk reads
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initIMU();
        initHardware();
        drivetrain = new Drivetrain(hwMap, telemetry, this);
        intake = new Intake(hwMap, telemetry, this);
        slide = new LinearSlide(hwMap, telemetry, this);
        rig = new Rigging(hwMap, telemetry, this);
        launcher = new AirplaneLauncher(hwMap, telemetry, this);

        subsystems = Arrays.asList(drivetrain, intake, slide, rig, launcher);
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
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    public void initHardware() {
        // Airplane Launcher Subsystem
        airplaneLauncherServo = hwMap.servo.get("AirplaneLauncher");
        airplaneLauncherServo.setPosition(0);

        // Drivetrain Subsystem
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");

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
        intakeMotor = hwMap.get(DcMotorEx.class, "Intake");

        // Linear Slide Subsystem
        linearSlideMotor = hwMap.get(DcMotorEx.class, "LinearSlide");
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Rigging Subsystem
        leftRigServo = hwMap.servo.get("LeftRiggingServo");
        rightRigServo = hwMap.servo.get("RightRiggingServo");
        leftRigMotor = hwMap.get(DcMotorEx.class, "LeftRigging");
        rightRigMotor = hwMap.get(DcMotorEx.class, "RightRigging");
        leftRigMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRigMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      leftRigInitialPos = leftRigServo.getPosition();
//      rightRigInitialPos = rightRigServo.getPosition();
    }

    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
