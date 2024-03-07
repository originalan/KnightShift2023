package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ArmSettings;
import org.firstinspires.ftc.teamcode.util.BulkReading;
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
        RIGHT_CLAW_OPEN,
        LEFT_CLAW_OPEN
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
            telemetry.addData("   Servo Left Position", BulkReading.pClawLeftServo);
            telemetry.addData("   Servo Right Position", BulkReading.pClawRightServo);
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
            case RIGHT_CLAW_OPEN:
                robot.clawRightServo.setPosition(ArmSettings.CLAW_RIGHT_OPEN);
                robot.clawLeftServo.setPosition(ArmSettings.CLAW_LEFT_CLOSE);
                break;
            case LEFT_CLAW_OPEN:
                robot.clawRightServo.setPosition(ArmSettings.CLAW_RIGHT_CLOSE);
                robot.clawLeftServo.setPosition(ArmSettings.CLAW_LEFT_OPEN);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void openClawBoth() {
        robot.clawRightServo.setPosition(ArmSettings.CLAW_RIGHT_OPEN);
        robot.clawLeftServo.setPosition(ArmSettings.CLAW_LEFT_OPEN);
    }

    public void closeClawBoth() {
        robot.clawRightServo.setPosition(ArmSettings.CLAW_RIGHT_CLOSE);
        robot.clawLeftServo.setPosition(ArmSettings.CLAW_LEFT_CLOSE);
    }

}
