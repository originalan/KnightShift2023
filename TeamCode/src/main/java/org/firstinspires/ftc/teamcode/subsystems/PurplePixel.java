package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

public class PurplePixel extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum PurplePixelState {
        HOLD,
        DROP
    }
    public PurplePixelState purplePixelState = PurplePixelState.HOLD;

    public PurplePixel(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;


    }

    @Override
    public void addTelemetry() {

    }

    @Override
    public void update() {
        switch (purplePixelState) {
            case HOLD:
                hold();
                break;
            case DROP:
                drop();
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void hold() {
        robot.purplePixelServo.setPosition(RobotSettings.PURPLE_PIXEL_SERVO_STARTING_POSITION);
    }

    public void drop() {
        robot.purplePixelServo.setPosition(RobotSettings.PURPLE_PIXEL_SERVO_ENDING_POSITION);
    }

}
