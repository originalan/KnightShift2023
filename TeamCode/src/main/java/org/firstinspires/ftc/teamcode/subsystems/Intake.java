package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private HardwareMap hwMap;
    private DcMotorEx motor;
    private Telemetry telemetry;

    public Intake(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motor = hwMap.get(DcMotorEx.class, "Intake");

    }

    public void intakeTelemetry() {

        telemetry.addData("Intake Power", motor.getPower());

    }

    public void moveForwards() {

        motor.setPower(1);

    }

    public void moveBackwards() {

        motor.setPower(-1);

    }

    public void turnOff() {

        motor.setPower(0);

    }

}
