package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Rigging extends Subsystem {

    private HardwareMap hwMap;
    private Servo leftRigServo;
    private Servo rightRigServo;

    private DcMotorEx leftRigMotor;
    private DcMotorEx rightRigMotor;

    private Telemetry telemetry;

    private final double leftRigInitialPos;
    private final double rightRigInitialPos;

    public enum RiggingState {
        NO_RIG,
        RIG
    }

    public RiggingState riggingState = RiggingState.NO_RIG;

    public Rigging(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        leftRigServo = hwMap.servo.get("LeftRiggingServo");
        rightRigServo = hwMap.servo.get("RightRiggingServo");

        leftRigMotor = hwMap.get(DcMotorEx.class, "LeftRigging");
        rightRigMotor = hwMap.get(DcMotorEx.class, "RightRigging");

//        leftRigInitialPos = leftRigServo.getPosition();
//        rightRigInitialPos = rightRigServo.getPosition();
        leftRigInitialPos = 0;
        rightRigInitialPos = 0;

    }

    @Override
    public void addTelemetry() {

        telemetry.addData("Left/Right Rig Servo Position", "%4.2f, %4.2f", leftRigServo.getPosition(), rightRigServo.getPosition());

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

        leftRigServo.setPosition(leftRigInitialPos + 0.25);
        rightRigServo.setPosition(rightRigInitialPos + 0.25);

        // Some code with the motors that pull the strings at the same time

    }

    public void noHang() {

        leftRigServo.setPosition(leftRigInitialPos);
        rightRigServo.setPosition(rightRigInitialPos);

    }

}
