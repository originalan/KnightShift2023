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

    private double leftRigInitialPos;
    private double rightRigInitialPos;

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

    public void addTelemetry() {

        telemetry.addData("Left/Right Rig Servo Position", "%4.2f, %4.2f", leftRigServo.getPosition(), rightRigServo.getPosition());

    }

    public void hang() {

        leftRigServo.setPosition(leftRigInitialPos + 0.25);
        rightRigServo.setPosition(rightRigInitialPos + 0.25);

        // Some code with the motors that pull the strings at the same time

    }

    public void undoHang() {

        leftRigServo.setPosition(leftRigInitialPos);
        rightRigServo.setPosition(rightRigInitialPos);

    }

}
