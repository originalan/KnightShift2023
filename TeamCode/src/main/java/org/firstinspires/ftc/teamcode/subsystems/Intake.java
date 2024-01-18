package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * Intake is a Subsystem representing all intake hardware movement
 */
public class Intake extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum IntakeState {
        OFF,
        FORWARD,
        REVERSE,
        HOLDING_PIXEL
    }

    public IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Intake");
        telemetry.addData("   Motor Power", "%4.2f", robot.intakeMotor.getPower());
    }

    @Override
    public void update() {
        switch (intakeState) {
            case OFF:
                turnOff();
                break;
            case FORWARD:
                moveForwards();
                break;
            case REVERSE:
                moveBackwards();
                break;
            case HOLDING_PIXEL:
                holdPixel();
                break;
        }
    }

    @Override
    public void stop() {
        turnOff();
    }

    public void moveForwards() {
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setPower(RobotSettings.INTAKE_FORWARD_MOTOR_SPEED);
    }

    public void moveBackwards() {
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setPower(RobotSettings.INTAKE_REVERSE_MOTOR_SPEED);
    }

    public void turnOff() {
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setPower(0);
    }

    public void holdPixel() {

        // Code is not as efficient as possible, may tinker later
        int currentPos = robot.intakeMotor.getCurrentPosition();
        int modFactor = (int)(PIDFControl.motorEncoderTicks / 2);

        int remainder = currentPos % modFactor;

        int targetPos = currentPos - remainder;

        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeMotor.setTargetPosition(targetPos);
        robot.intakeMotor.setPower(0.2);

    }

}
