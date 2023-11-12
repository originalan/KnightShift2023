package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Rigging extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    private final double leftRigInitialPos = 0;
    private final double rightRigInitialPos = 0;

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
        telemetry.addData("Left/Right Rig Servo Position", "%4.2f, %4.2f", robot.leftRigServo.getPosition(), robot.rightRigServo.getPosition());
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
        }
    }

    @Override
    public void stop() {
        noHang();
    }

    public void hang() {
        robot.leftRigServo.setPosition(leftRigInitialPos + 0.25);
        robot.rightRigServo.setPosition(rightRigInitialPos + 0.25);
        // Motor string code is handled elsewhere
    }

    public void noHang() {
        robot.leftRigServo.setPosition(leftRigInitialPos);
        robot.rightRigServo.setPosition(rightRigInitialPos);
    }

}
