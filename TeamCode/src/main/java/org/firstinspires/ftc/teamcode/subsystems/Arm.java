package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public int encoderPosition = 0;

    public enum ArmState {
        AT_REST, // no power
        BOTTOM_CLAW_UP,
        BOTTOM_CLAW_DOWN,
        GO_TO_POSITION,
        AUTO_YELLOW_POS,
        AUTO_PIXEL_STACK_POS_1,
        AUTO_PIXEL_STACK_POS_2,
        PIVOT_TEST,
        NOTHING
    }

    public ArmState armState = ArmState.BOTTOM_CLAW_UP;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.pid = new PIDFControl();

        JVBoysSoccerRobot.initialArmPosition = robot.armLeftMotor.getCurrentPosition();
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.ARM) {
            telemetry.addLine("Arm");
            telemetry.addData("   Motor Position Encoder Value", "%d", robot.armLeftMotor.getCurrentPosition());
            telemetry.addData("    Left/Right Pivot Servo Pos", "%.3f, %.3f", robot.clawPivotLeftServo.getPosition(), robot.clawPivotRightServo.getPosition());
        }
    }

    @Override
    public void update() {
        switch (armState) {
            case GO_TO_POSITION:
                noEncoders();
                setArmPower(targetPower);
                break;
            case AT_REST:
                noEncoders();
                setArmPower(0);
                break;
            case NOTHING:
                break;
            case BOTTOM_CLAW_UP:
                robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_REST);
                setArmEncoderPosition(ArmSettings.positionBottom);
                break;
            case BOTTOM_CLAW_DOWN:
                robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_GROUND);
                setArmEncoderPosition(ArmSettings.positionBottom);
                break;
            case AUTO_YELLOW_POS:
                robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_YELLOW); // -0.5 / 3 + 200/180
                setArmEncoderPosition( ArmSettings.positionYellowPixel );
                break;
            case AUTO_PIXEL_STACK_POS_1: // for redclose1 and redclose2, pick up 2 white pixels
                robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_PIXELSTACK1);
                setArmEncoderPosition( ArmSettings.positionPixelStack1); // supposed to go up ~1.5 inches
                break;
            case AUTO_PIXEL_STACK_POS_2: // for redfar1, pick up 1 white pixel
                robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_PIXELSTACK2);
                setArmEncoderPosition( ArmSettings.positionPixelStack2); // supposed to go up ~2.0 inches
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
        double pow = pid.calculate(encoderPos, robot.armLeftMotor.getCurrentPosition(), false);
        robot.armLeftMotor.setPower(pow);
        robot.armRightMotor.setPower(pow);
    }

    public void noEncoders() {
        robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
