package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * AirplaneLauncher is a Subsystem representing all airplane launcher hardware movement
 */
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
        telemetry.addLine("Airplane Launcher");
        telemetry.addData("   Servo Current Position", "%4.2f", robot.airplaneLauncherServo.getPosition());
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
        robot.airplaneLauncherServo.setPosition(RobotSettings.LAUNCHER_POSITION_OUT);
    }

    public void notYet() {
        robot.airplaneLauncherServo.setPosition(RobotSettings.LAUNCHER_POSITION_IN);
    }

}
