package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Rigging {

    private HardwareMap hwMap;
    private Servo leftRig;
    private Servo rightRig;
    private Telemetry telemetry;

    private double leftRigInitialPos;
    private double rightRigInitialPos;

    public Rigging(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        leftRig = hwMap.servo.get("LeftRiggingServo");
        rightRig = hwMap.servo.get("RightRiggingServo");

        leftRigInitialPos = leftRig.getPosition();
        rightRigInitialPos = rightRig.getPosition();

    }

    public void rigTelemetry() {

        telemetry.addData("Left/Right Rig Servo Position", "%4.2f, %4.2f", leftRig.getPosition(), rightRig.getPosition());

    }

    public void hang() {

        leftRig.setPosition(leftRigInitialPos + 0.25);
        rightRig.setPosition(rightRigInitialPos + 0.25);

    }

    public void undoHang() {

        leftRig.setPosition(leftRigInitialPos);
        rightRig.setPosition(rightRigInitialPos);

    }

}
