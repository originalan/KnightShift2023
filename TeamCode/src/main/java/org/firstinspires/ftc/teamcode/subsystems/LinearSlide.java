package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDControl;

public class LinearSlide {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private PIDControl pid;

    private DcMotorEx motor;

    public LinearSlide(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motor = hwMap.get(DcMotorEx.class, "LinearSlide");

    }

    public void extendSlide() {



    }

    public void addTelemetry() {

        telemetry.addData("Linear Slide Position Encoder Value", motor.getCurrentPosition());

    }

}
