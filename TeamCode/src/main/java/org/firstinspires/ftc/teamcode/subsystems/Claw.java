package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.teamcode.util.UseTelemetry;

/**
 * Intake is a Subsystem representing all intake hardware movement
 */
public class Claw extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum ClawState {
        NOTHING,
        BOTH_CLOSED,
        BOTH_OPEN,
        LEFT_OPEN,
        RIGHT_OPEN
    }

    public ClawState clawState = ClawState.BOTH_CLOSED;

    public Claw(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.CLAW) {
            telemetry.addLine("Intake");
            telemetry.addData("   Servo Left Position", robot.clawLeftServo.getPosition());
            telemetry.addData("   Servo Right Position", robot.clawRightServo.getPosition());
        }
    }

    @Override
    public void update() {
        switch (clawState) {
            case NOTHING:
                break;
            case BOTH_CLOSED:
                closeClawBoth();
                break;
            case BOTH_OPEN:
                openClawBoth();
                break;
            case LEFT_OPEN:
                robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_OPEN);
                robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_CLOSE);
                break;
            case RIGHT_OPEN:
                robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_CLOSE);
                robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_OPEN);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void openClawBoth() {
        robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_CLOSE);
        robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_CLOSE);
    }

    public void closeClawBoth() {
        robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_OPEN);
        robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_OPEN);
    }

}
