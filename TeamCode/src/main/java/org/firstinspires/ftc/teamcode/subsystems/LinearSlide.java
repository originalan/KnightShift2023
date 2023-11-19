package org.firstinspires.ftc.teamcode.subsystems;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

public class LinearSlide extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum SlideState {
        OFF,
        HOLDING_PIXEL,
        BRING_ARM_IN_PLACE,
        BRING_ARM_BACK,
        RELEASE_PIXEL
    }

    public SlideState slideState = SlideState.OFF;
    public boolean isBusy;

    public LinearSlide(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        isBusy = false;

        robot.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Linear Slide");
        telemetry.addData("   Motor Position Encoder Value", "%d", robot.linearSlideMotor.getCurrentPosition());
        telemetry.addData("   Servo Position Value" , "%4.2f", robot.linearSlideServo.getPosition());
    }

    @Override
    public void update() {
        switch (slideState) {
            case OFF:
                robot.linearSlideMotor.setPower(0);
                robot.linearSlideServo.setPosition(RobotSettings.OUTTAKE_SERVO_CLAW_STARTING_POSITION);
                break;
            case HOLDING_PIXEL:
                robot.linearSlideMotor.setPower(0);
                robot.linearSlideServo.setPosition(RobotSettings.OUTTAKE_SERVO_CLAW_HOLDING_POSITION);
                break;
            case BRING_ARM_IN_PLACE:
                robot.linearSlideServo.setPosition(RobotSettings.OUTTAKE_SERVO_CLAW_HOLDING_POSITION);
                robot.linearSlideMotor.setPower(RobotSettings.OUTTAKE_MOTOR_POWER);
                break;
            case RELEASE_PIXEL:
                robot.linearSlideMotor.setPower(0);
                robot.linearSlideServo.setPosition(RobotSettings.OUTTAKE_SERVO_CLAW_RELEASE_POSITION);
                break;
            case BRING_ARM_BACK:
                robot.linearSlideMotor.setPower(RobotSettings.OUTTAKE_MOTOR_POWER);
                robot.linearSlideServo.setPosition(RobotSettings.OUTTAKE_SERVO_CLAW_RELEASE_POSITION);
                break;
        }
    }

    @Override
    public void stop() {
        // Unextend the slide
    }

}
