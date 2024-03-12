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
    private MotionProfile mp;
    private SuperController superController;

    public static double MAX_POWER = 0.45;
    public ElapsedTime motionProfileTime = new ElapsedTime();
    private double maxVelocity = 0;
    private int STARTING_POS = 0;
    private int ENDING_POS = 0;
    private double previousPower = 5;
    private double previousRefPos = 100000;
    private double previousCurrentPos = 100000;

    public enum ArmState {
        AUTO_PIXEL_STACK_POS_1,
        AUTO_PIXEL_STACK_POS_2,
        NOTHING,
        MOTION_PROFILE,
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
        this.mp = new MotionProfile();
        this.superController = new SuperController(telemetry);

        noEncoders();

        JVBoysSoccerRobot.initialArmPosition = BulkReading.pArmLeftMotor;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.ARM) {
            telemetry.addLine("Arm");
            telemetry.addData("    Arm Motor Position Encoder Value", "%d", BulkReading.pArmLeftMotor);
            telemetry.addData("Arm State", armState);
            telemetry.addData("    Max velocity", maxVelocity);
            telemetry.addData("    Left/Right Pivot Servo Pos", "%.3f, %.3f", BulkReading.pClawPivotLeftServo, BulkReading.pClawPivotRightServo);
        }
    }

    @Override
    public void update() {
        if (Math.abs(BulkReading.vArmLeftMotor) > maxVelocity) {
            maxVelocity = BulkReading.vArmLeftMotor;
        }
        switch (armState) {
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
                mp.updateState(motionProfileTime.seconds());
                double refPos = mp.getInstantPosition();
                double refVel = mp.getInstantVelocity();
                double refAcl = mp.getInstantAcceleration();

                telemetry.addData("MP TIME", motionProfileTime.seconds());
                telemetry.addData("Reference Position", refPos);
                telemetry.addData("Reference Velocity", refVel);
                telemetry.addData("Reference Acceleration", refAcl);

                double pidPower = 0;
                double fullstate = 0;
                double output = 0;
//                if ( !(previousCurrentPos == BulkReading.pArmLeftMotor && previousRefPos == refPos) ) {
////                pidPower = superController.calculatePID(refPos, BulkReading.pArmLeftMotor);
//                    fullstate = superController.fullstateCalculate(refPos, refVel, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
////                fullstate = superController.fullstateCalculate(refPos, refVel, refAcl, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
//                    double f_g = superController.positionalFeedforward(refPos);
//                    double k_va = superController.kvkaFeedforward(refVel, refAcl);
//
//                    output = pidPower + fullstate + f_g + k_va; // PID + gravity positional feedforward + velocity and acceleration feedforward
//                    setArmPower(output);
//                }
                pidPower = superController.calculatePID(refPos, BulkReading.pArmLeftMotor);
                fullstate = superController.fullstateCalculate(refPos, refVel, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
//                fullstate = superController.fullstateCalculate(refPos, refVel, refAcl, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
                double f_g = superController.positionalFeedforward(refPos);
                double k_va = superController.kvkaFeedforward(refVel, refAcl);

                output = pidPower + fullstate + f_g + k_va; // PID + gravity positional feedforward + velocity and acceleration feedforward
                setArmPower(output);

                previousCurrentPos = BulkReading.pArmLeftMotor;
                previousRefPos = refPos;
                break;
            case PIDF_TEST:
                double pid = 0;
                double Fg = 0;
                double Kva = 0;
                double full_state = 0;

                double t = ArmTestPIDF.targetPos;

                if (ArmTestPIDF.feedforward_g) {
                    Fg = superController.positionalFeedforward(t);
                }
                if (ArmTestPIDF.fullstate) {
                    full_state = superController.fullstateCalculate(t, ArmTestPIDF.targetVelocity, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
//                    full_state = superController.fullstateCalculate(t, ArmTestPIDF.targetVelocity, ArmTestPIDF.targetAcceleration, BulkReading.pArmLeftMotor, BulkReading.vArmLeftMotor);
                }
                if (ArmTestPIDF.pid) {
                    pid = superController.calculatePIDGravity(t, BulkReading.pArmLeftMotor);
                }
                if (ArmTestPIDF.feedforward_kvka) {
                    Kva = superController.kvkaFeedforward(ArmTestPIDF.targetVelocity, ArmTestPIDF.targetAcceleration);
                }

                double a = pid + Fg + Kva + full_state;
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
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }
        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }

        if (power != previousPower) {
            robot.armLeftMotor.setPower(power);
            robot.armRightMotor.setPower(power);
        }


        previousPower = power;
    }

    public void setArmEncoderPosition(int encoderPos) {
        double pow = superController.calculatePIDGravity(encoderPos, BulkReading.pArmLeftMotor);

        if (pow > MAX_POWER) {
            pow = MAX_POWER;
        }
        if (pow < -MAX_POWER) {
            pow = -MAX_POWER;
        }

        robot.armLeftMotor.setPower(pow);
        robot.armRightMotor.setPower(pow);
    }

    public void setPivotServoPosition(double position) {
        robot.clawPivotLeftServo.setPosition(position);
        robot.clawPivotRightServo.setPosition(position);
    }

    public void noEncoders() {
        robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotionProfile(int targetPosition) {
        noEncoders();
        motionProfileTime.reset();
        mp.setStartingTime(motionProfileTime.seconds());

        STARTING_POS = BulkReading.pArmLeftMotor;
        ENDING_POS = targetPosition;

        mp.setProfile(STARTING_POS, ENDING_POS);
    }

    public MotionProfile getMP() {
        return mp;
    }

}
