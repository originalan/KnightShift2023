package org.firstinspires.ftc.teamcode.subsystems;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDControl;

public class LinearSlide extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum SlideState {
        OFF,
        HALF_EXTENDED,
        FULLY_EXTENDED,
        VARIABLE_EXTENSION
    }

    public SlideState slideState = SlideState.OFF;

    public LinearSlide(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
//        telemetry.addLine("Linear Slide");
        telemetry.addData("   Motor Position Encoder Value", "%d", robot.linearSlideMotor.getCurrentPosition());
    }

    @Override
    public void update() {
        switch (slideState) {
            case OFF:
                // Do stuff
                break;
            case HALF_EXTENDED:
                break;
            case FULLY_EXTENDED:
                break;
            case VARIABLE_EXTENSION:
                break;
        }
    }

    @Override
    public void stop() {
        // Unextend the slide
    }

}
