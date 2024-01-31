package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * Intake is a Subsystem representing all intake hardware movement
 */
public class Claw extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum ClawState {
        NA
    }

    public ClawState clawState = ClawState.NA;

    public Claw(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Intake");
        telemetry.addData("   Servo Left Position", robot.clawLeftServo.getPosition());
        telemetry.addData("   Servo Right Position", robot.clawRightServo.getPosition());
    }

    @Override
    public void update() {
        switch (clawState) {
            case NA:
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void openClawBoth() {
        robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_OPEN);
        robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_OPEN);
    }

    public void closeClawBoth() {
        robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_CLOSED);
        robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_CLOSED);
    }

}
