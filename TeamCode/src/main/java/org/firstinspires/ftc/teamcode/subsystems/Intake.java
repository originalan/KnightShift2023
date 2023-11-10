package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {

    private HardwareMap hwMap;
    private DcMotorEx motor;
    private Telemetry telemetry;

    public enum IntakeState {
        OFF,
        FORWARDS,
        BACKWARDS
    }

    public IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motor = hwMap.get(DcMotorEx.class, "Intake");

    }

    @Override
    public void addTelemetry() {

        telemetry.addData("Intake Power", motor.getPower());

    }

    @Override
    public void update() {

        switch (intakeState) {

            case OFF:
                turnOff();
                break;

            case FORWARDS:
                moveForwards();
                break;

            case BACKWARDS:
                moveBackwards();
                break;

        }

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
