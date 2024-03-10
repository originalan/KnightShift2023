package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

/**
 * Intake is a Subsystem representing all intake hardware movement
 */
public class Claw extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    public boolean leftClosed = true, rightClosed = true;

    public enum ClawState {
        NOTHING,
        BOTH_CLOSED,
        BOTH_OPEN,
        RIGHT_CLAW_OPEN,
        LEFT_CLAW_OPEN,
        AUTO_DETECT
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
                closeClawLeft();
                closeClawRight();
                break;
            case BOTH_OPEN:
                openClawLeft();
                openClawRight();
                break;
            case RIGHT_CLAW_OPEN:
                openClawRight();
                closeClawLeft();
                break;
            case LEFT_CLAW_OPEN:
                openClawLeft();
                closeClawRight();
                break;
            case AUTO_DETECT:
                autoDetect();
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void openClawLeft() {
        leftClosed = false;
        robot.clawLeftServo.setPosition(ArmSettings.CLAW_LEFT_OPEN);
    }
    public void openClawRight() {
        rightClosed = false;
        robot.clawRightServo.setPosition(ArmSettings.CLAW_RIGHT_OPEN);
    }
    public void closeClawLeft() {
        leftClosed = true;
        robot.clawLeftServo.setPosition(ArmSettings.CLAW_LEFT_CLOSE);
    }
    public void closeClawRight() {
        rightClosed = true;
        robot.clawRightServo.setPosition(ArmSettings.CLAW_RIGHT_CLOSE);
    }

    public void autoDetect() {
        double left = robot.clawDLeft.getDistance(DistanceUnit.INCH);
        double right = robot.clawDRight.getDistance(DistanceUnit.INCH);

        if (left < RobotSettings.CLAW_LEFT_THRESHOLD) {
            closeClawLeft();
        }else {
            openClawLeft();
        }

        if (right < RobotSettings.CLAW_RIGHT_THRESHOLD) {
            closeClawRight();
        }else {
            openClawRight();
        }
    }

}
