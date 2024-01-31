package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * Rigging is a Subsystem representing all rigging/hanging hardware movement
 */
public class Rigging extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum RiggingState {
        NO_RIG,
        RIG
    }

    public RiggingState riggingState = RiggingState.NO_RIG;

    public Rigging(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Rigging");
        telemetry.addData("   Left/Right Servo Position", "%4.2f, %4.2f", robot.rigLeftServo.getPosition(), robot.rigRightServo.getPosition());
        telemetry.addData("   Left/Right Motor Position", "%d, %d", robot.rigLeftMotor.getCurrentPosition(), robot.rigRightMotor.getCurrentPosition());
    }

    @Override
    public void update() {
//        switch (riggingState) {
//            case NO_RIG:
//                noHang();
//                break;
//            case RIG:
//                hang();
//                break;
//        }
    }

    @Override
    public void stop() {
        noHang();
    }

    public void hang() {
        robot.rigLeftServo.setPosition(RobotSettings.RIGGING_LEFT_REST + RobotSettings.RIGGING_MOVE_SERVO);
        robot.rigRightServo.setPosition(RobotSettings.RIGGING_RIGHT_REST + RobotSettings.RIGGING_MOVE_SERVO);
        // Motor string code is handled elsewhere
    }

    public void hang(double increment) {
        robot.rigLeftServo.setPosition(RobotSettings.RIGGING_LEFT_REST + increment);
        robot.rigRightServo.setPosition(RobotSettings.RIGGING_RIGHT_REST + increment);
    }

    public void noHang() {
        robot.rigLeftServo.setPosition(RobotSettings.RIGGING_LEFT_REST);
        robot.rigRightServo.setPosition(RobotSettings.RIGGING_RIGHT_REST);
    }

}
