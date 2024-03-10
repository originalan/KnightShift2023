package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.ArmTestPIDF;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;
import org.firstinspires.ftc.teamcode.util.SuperController;

/**
 * DeliveryArm is a Subsystem representing all delivery arm hardware movement
 */
@Config
public class Arm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;
    private MotionProfile mp;
    private SuperController superController;

    public double targetPower = 0; // used purely for GO_TO_POSITION arm state, PIDF test
    public int encoderGoalPosition = 0;
    public int goalDistance = 0;
    public ElapsedTime motionProfileTime = new ElapsedTime();
    public int armPositionMP = 0;
    public static double MAX_A = 250; // in encoder ticks / second^2
    public static double MAX_V = 250; // in encoder ticks / second
    private double maxVelocity = 0;
    private int STARTING_POS = 0;
    private int ENDING_POS = 0;

    public enum ArmState {
        BOTTOM_CLAW_UP,
        BOTTOM_CLAW_DOWN,
        GO_TO_POSITION,
        AUTO_YELLOW_POS,
        AUTO_PIXEL_STACK_POS_1,
        AUTO_PIXEL_STACK_POS_2,
        NOTHING,
        MOTION_PROFILE,
        MOTION_PROFILE_TEST,
        PIDF_TEST
    }

    public enum PivotState {
        NOTHING,
        REST,
        GROUND,
        AUTO_CALIBRATE
    }

    public ArmState armState = ArmState.NOTHING;
    public PivotState pivotState = PivotState.NOTHING;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.pid = new PIDFControl();
        this.mp = new MotionProfile();
        this.superController = new SuperController();

        noEncoders();

        JVBoysSoccerRobot.initialArmPosition = BulkReading.pArmLeftMotor;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.ARM) {
            telemetry.addLine("Arm");
            telemetry.addData("    Arm Motor Position Encoder Value", "%d", BulkReading.pArmLeftMotor);
            telemetry.addData("    Max velocity", maxVelocity);
            telemetry.addData("    Left/Right Pivot Servo Pos", "%.3f, %.3f", BulkReading.pClawPivotLeftServo, BulkReading.pClawPivotRightServo);
        }
    }

    @Override
    public void update() {
        if (Math.abs(BulkReading.vArmLeftMotor) > maxVelocity) {
            maxVelocity = BulkReading.vArmLeftMotor;
        }
        if (encoderGoalPosition < 0) {
            encoderGoalPosition = 0;
        }
        switch (armState) {
            case GO_TO_POSITION:
                noEncoders();
                setArmPower(targetPower);
                break;
            case BOTTOM_CLAW_UP:
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_REST);
                setArmEncoderPosition(ArmSettings.positionBottom);
                break;
            case BOTTOM_CLAW_DOWN:
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_GROUND);
                setArmEncoderPosition(ArmSettings.positionBottom);
                break;
            case AUTO_YELLOW_POS:
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_YELLOW); // -0.5 / 3 + 200/180
                setArmEncoderPosition( ArmSettings.positionYellowPixel );
                break;
            case AUTO_PIXEL_STACK_POS_1: // for redclose1 and redclose2, pick up 2 white pixels
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_PIXELSTACK1);
                setArmEncoderPosition( ArmSettings.positionPixelStack1); // supposed to go up ~1.5 inches
                break;
            case AUTO_PIXEL_STACK_POS_2: // for redfar1, pick up 1 white pixel
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_PIXELSTACK2);
                setArmEncoderPosition( ArmSettings.positionPixelStack2); // supposed to go up ~2.0 inches
                break;
            case NOTHING:
                break;
            case MOTION_PROFILE:
                noEncoders();
                double instantTargetPos = pid.motionProfile(MAX_A, MAX_V, goalDistance, motionProfileTime.seconds()) + armPositionMP;
                double power = pid.calculatePID(instantTargetPos, BulkReading.pArmLeftMotor, false);
                double ff = pid.calculateFeedforward(instantTargetPos, true);
                setArmPower(power + ff);
                break;
            case MOTION_PROFILE_TEST:
                mp.updateState(motionProfileTime.seconds());
                double refPos = mp.getInstantPosition();
                double refVel = mp.getInstantVelocity();
                double refAcl = mp.getInstantAcceleration();

                double pidPower = 0;
                pidPower = superController.calculatePID(refPos, BulkReading.pArmLeftMotor);
                double fullstate = 0;
//                fullstate = superController.fullstateCalculate(refPos, refVel, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
//                fullstate = superController.fullstateCalculate(refPos, refVel, refAcl, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
                double f_g = superController.positionalFeedforward(refPos);
                double k_va = superController.kvkaFeedforward(refVel, refAcl);

                double output = pidPower + fullstate + f_g + k_va; // PID + gravity positional feedforward + velocity and acceleration feedforward
                setArmPower(output);
                break;
            case PIDF_TEST:
                double pid = 0;
                double fg = 0;
                double kva = 0;
                double full_state = 0;

                double t = ArmTestPIDF.targetPos;

                if (ArmTestPIDF.feedforward_g) {
                    fg = superController.positionalFeedforward(t);
                }
                if (ArmTestPIDF.fullstate) {
                    full_state = superController.fullstateCalculate(t, ArmTestPIDF.targetVelocity, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
//                    full_state = superController.fullstateCalculate(t, ArmTestPIDF.targetVelocity, ArmTestPIDF.targetAcceleration, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
                }
                if (ArmTestPIDF.pid) {
                    pid = superController.calculatePID(t, BulkReading.pArmLeftMotor);
                }
                if (ArmTestPIDF.feedforward_kvka) {
                    kva = superController.kvkaFeedforward(ArmTestPIDF.targetVelocity, ArmTestPIDF.targetAcceleration);
                }

                double a = pid + fg + kva + full_state;
                setArmPower(a);
                break;
        }

        switch (pivotState) {
            case NOTHING:
                break;
            case REST:
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_REST);
                break;
            case GROUND:
                setPivotServoPosition(ArmSettings.ARM_PIVOT_SERVO_GROUND);
                break;
            case AUTO_CALIBRATE:
                if (BulkReading.pArmLeftMotor > 375) {
                    setPivotServoPosition( ArmSettings.ARM_PIVOT_SERVO_REST + (1.0/3.0) + 0.10 + ( BulkReading.pArmLeftMotor / 1120.0 ) );
                }
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void setArmPower(double power) {
        robot.armLeftMotor.setPower(power);
        robot.armRightMotor.setPower(power);
    }

    public void setArmEncoderPosition(int encoderPos) {
        double pow = pid.calculatePID(encoderPos, BulkReading.pArmLeftMotor, false);
        robot.armLeftMotor.setPower(pow);
        robot.armRightMotor.setPower(pow);
    }

    public void setPivotServoPosition(double position) {
//        robot.clawPivotLeftServo.setPosition(position);
        robot.clawPivotRightServo.setPosition(position);
    }

    public void noEncoders() {
        robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotionProfile() {

        goalDistance = (int)encoderGoalPosition - BulkReading.pArmLeftMotor;
        armPositionMP = BulkReading.pArmLeftMotor;
        motionProfileTime.reset();

    }
    public void setMotionProfileTest(int targetPosition) {
        noEncoders();
        motionProfileTime.reset();
        mp.setStartingTime(motionProfileTime.seconds());

        STARTING_POS = BulkReading.pArmLeftMotor;
        ENDING_POS = targetPosition;

        mp.setProfile(STARTING_POS, ENDING_POS);
    }

}
