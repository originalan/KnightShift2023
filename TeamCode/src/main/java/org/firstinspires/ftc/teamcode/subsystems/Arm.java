package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmSettings;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.UseTelemetry;

/**
 * DeliveryArm is a Subsystem representing all delivery arm hardware movement
 */
@Config
public class Arm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;

    public double targetPower = 0; // used purely for GO_TO_POSITION arm state, PIDF test
    public int encoderGoalPosition = 0;
    public int goalDistance = 0;
    public ElapsedTime motionProfileTime = new ElapsedTime();
    public int armPositionMP = 0;
    private double instantTargetPos = 0;
    public static double MAX_A = 175; // in encoder ticks / second^2
    public static double MAX_V = 250; // in encoder ticks / second
    private double maxVelocity = 0;

    public enum ArmState {
        BOTTOM_CLAW_UP,
        BOTTOM_CLAW_DOWN,
        GO_TO_POSITION,
        AUTO_YELLOW_POS,
        AUTO_PIXEL_STACK_POS_1,
        AUTO_PIXEL_STACK_POS_2,
        NOTHING,
        PIVOT_SERVO_MOVE,
        MOTION_PROFILE
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

        noEncoders();

        JVBoysSoccerRobot.initialArmPosition = robot.armLeftMotor.getCurrentPosition();
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.ARM) {
            telemetry.addLine("Arm");
            telemetry.addData("    Arm Motor Position Encoder Value", "%d", robot.armLeftMotor.getCurrentPosition());
            telemetry.addData("    Arm Motor Power", robot.armLeftMotor.getPower());
            telemetry.addData("    Max velocity", maxVelocity);
            telemetry.addData("    Left/Right Pivot Servo Pos", "%.3f, %.3f", robot.clawPivotLeftServo.getPosition(), robot.clawPivotRightServo.getPosition());
        }
    }

    @Override
    public void update() {
        if (Math.abs(robot.armLeftMotor.getVelocity()) > maxVelocity) {
            maxVelocity = robot.armLeftMotor.getVelocity();
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
            case PIVOT_SERVO_MOVE:
                // NEED TO WRITE CODE
                // 1.0 is 180 degrees
                // 1120 is for 360 degrees
                // 1120 / 2 is for 180 degrees
                // 1120/2 / 180.0 = x / 1.0
                // solve for x, x + initial servo pos = arm pivot servo pos
                setPivotServoPosition( ArmSettings.ARM_PIVOT_SERVO_REST + (1.0/3.0) + ( robot.armLeftMotor.getCurrentPosition() / 1120.0 ) );
                break;
            case MOTION_PROFILE:
                instantTargetPos = pid.motionProfile(MAX_A, MAX_V, goalDistance, motionProfileTime.seconds()) + armPositionMP;
                double power = pid.calculatePID(instantTargetPos, robot.armLeftMotor.getCurrentPosition(), false);
                double ff = pid.calculateFeedforward(instantTargetPos, true);
                setArmPower(power + ff);
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
                if (robot.armLeftMotor.getCurrentPosition() > 375) {
                    setPivotServoPosition( ArmSettings.ARM_PIVOT_SERVO_REST - (1.0/3.0) + ( robot.armLeftMotor.getCurrentPosition() / 1120.0 ) );
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
        double pow = pid.calculatePID(encoderPos, robot.armLeftMotor.getCurrentPosition(), false);
        robot.armLeftMotor.setPower(pow);
        robot.armRightMotor.setPower(pow);
    }

    public void setPivotServoPosition(double position) {
        robot.clawPivotLeftServo.setPosition(position);
//        robot.clawPivotRightServo.setPosition(position);
    }

    public void noEncoders() {
        robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotionProfile() {

        goalDistance = (int)encoderGoalPosition - robot.armLeftMotor.getCurrentPosition();
        armPositionMP = robot.armLeftMotor.getCurrentPosition();
        motionProfileTime.reset();

    }

}
