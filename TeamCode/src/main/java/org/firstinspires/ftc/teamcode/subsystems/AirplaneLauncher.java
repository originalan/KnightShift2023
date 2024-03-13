package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

/**
 * AirplaneLauncher is a Subsystem representing all airplane launcher hardware movement
 */
public class AirplaneLauncher extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum LauncherState {
        AT_REST,
        ZONE_ONE_OR_BUST,
        NOTHING
    }
    public LauncherState launcherState = LauncherState.NOTHING;
    private LauncherState previousState = LauncherState.NOTHING;
    private int counter = 0;

    public AirplaneLauncher(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.LAUNCHER) {
            telemetry.addLine("Airplane Launcher");
            telemetry.addData("   Launcher Servo Current Position", "%4.2f", robot.launcherServo.getPosition());
            telemetry.addData("   Times states have changed", counter);
        }
    }

    @Override
    public void update() {
        if (launcherState == previousState) {
            counter++;
        }

        switch (launcherState) {
            case AT_REST:
                restFireServo();
                break;
            case ZONE_ONE_OR_BUST:
                releaseFireServo();
                break;
            case NOTHING:
                break;
        }

        previousState = launcherState;
    }

    @Override
    public void stop() {

    }

    public void releaseFireServo() {
        robot.launcherServo.setPosition(RobotSettings.LAUNCHER_FIRE_POSITION_FIRE);
    }

    public void restFireServo() {
        robot.launcherServo.setPosition(RobotSettings.LAUNCHER_FIRE_POSITION_REST);
    }
}
