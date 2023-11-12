package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDControl;

public class AirplaneLauncher extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum LauncherState {
        OFF,
        ZONE_ONE_OR_BUST
    }
    public LauncherState launcherState = LauncherState.OFF;

    public AirplaneLauncher(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addData("Airplane Launcher Servo Current Position", robot.airplaneLauncherServo.getPosition());
    }

    @Override
    public void update() {
        switch (launcherState) {
            case OFF:
                notYet();
                break;
            case ZONE_ONE_OR_BUST:
                ZONE_ONE_OR_BUST();
                break;
        }
    }

    @Override
    public void stop() {
        notYet();
    }

    public void ZONE_ONE_OR_BUST() {
        robot.airplaneLauncherServo.setPosition(1.0);
    }

    public void notYet() {
        robot.airplaneLauncherServo.setPosition(0);
    }

}
