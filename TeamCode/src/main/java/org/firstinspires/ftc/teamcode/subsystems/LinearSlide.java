package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDControl;

public class LinearSlide extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private PIDControl pid;

    private DcMotorEx motor;

    public enum SlideState {
        OFF,
        HALF_EXTENDED,
        FULLY_EXTENDED
    }

    public SlideState slideState = SlideState.OFF;

    public LinearSlide(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motor = hwMap.get(DcMotorEx.class, "LinearSlide");

    }

    public void update() {

        if (slideState == SlideState.FULLY_EXTENDED) {
            // Do stuff... (can also check previous slide state for code
        }else if (slideState == SlideState.HALF_EXTENDED) {

        }else if (slideState == SlideState.OFF) {

        }

    }

    public void addTelemetry() {

        telemetry.addData("Linear Slide Position Encoder Value", motor.getCurrentPosition());

    }

}
