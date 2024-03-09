package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

/**
 * Rigging is a Subsystem representing all rigging/hanging hardware movement
 */
public class Rigging extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum RiggingState {
        NO_RIG,
        RIG,
        NOTHING
    }

    public RiggingState riggingState = RiggingState.NOTHING;

    public Rigging(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.RIGGING) {
            telemetry.addLine("Rigging");
            telemetry.addData("   Left/Right Servo Position", "%4.2f, %4.2f", BulkReading.pRigLeftServo, BulkReading.pRigRightServo);
//            telemetry.addData("   Left/Right Motor Position", "%d, %d", robot.rigLeftMotor.getCurrentPosition(), robot.rigRightMotor.getCurrentPosition());
        }
    }

    @Override
    public void update() {
        switch (riggingState) {
            case NO_RIG:
                noHang();
                break;
            case RIG:
                hang();
                break;
            case NOTHING:
                break;
        }
    }

    @Override
    public void stop() {
        noHang();
    }

    public void hang() {
        robot.rigLeftServo.setPosition(RobotSettings.RIGGING_LEFT_TOP);
        robot.rigRightServo.setPosition(RobotSettings.RIGGING_RIGHT_TOP);
        // Motor string code is handled elsewhere
    }

    public void noHang() {
        robot.rigLeftServo.setPosition(RobotSettings.RIGGING_LEFT_REST);
        robot.rigRightServo.setPosition(RobotSettings.RIGGING_RIGHT_REST);
    }

    public void pullString(boolean isReversed) {
        double reversed = isReversed ? -1.0 : 1.0;
        //have u considered writing comments
        robot.rigLeftMotor.setPower(reversed);
        robot.rigRightMotor.setPower(reversed);
    }

}
