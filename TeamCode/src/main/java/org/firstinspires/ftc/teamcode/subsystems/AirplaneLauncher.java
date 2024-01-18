package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * AirplaneLauncher is a Subsystem representing all airplane launcher hardware movement
 */
public class AirplaneLauncher extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    private ElapsedTime timer = new ElapsedTime();
    private double currentTime = 0;
    private int timerCounter = 1;

    public enum LauncherState {
        AT_REST,
        ZONE_ONE_OR_BUST
    }
    public LauncherState launcherState = LauncherState.AT_REST;

    public AirplaneLauncher(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Airplane Launcher");
        telemetry.addData("   Servo Current Position", "%4.2f", robot.airplaneLauncherFireServo.getPosition());
    }

    @Override
    public void update() {
        switch (launcherState) {
            case AT_REST:
                launcherAtRest();
                adjustClamped();
                break;
            case ZONE_ONE_OR_BUST:
                if (timerCounter == 1) {
                    timer.reset();
                    timerCounter++;
                }
                currentTime = timer.seconds();
                adjustUnclamped();
                if (currentTime > 2.0) {
                    ZONE_ONE_OR_BUST();
                }
                break;
        }
    }

    @Override
    public void stop() {
        launcherAtRest();
    }

    public void ZONE_ONE_OR_BUST() {
        robot.airplaneLauncherFireServo.setPosition(RobotSettings.LAUNCHER_FIRE_POSITION_FIRE);
    }

    public void launcherAtRest() {
        robot.airplaneLauncherFireServo.setPosition(RobotSettings.LAUNCHER_FIRE_POSITION_REST);
    }

    public void adjustUnclamped() {
        robot.airplaneLauncherAdjustServo.setPosition(RobotSettings.LAUNCHER_ADJUST_POSITION_UNCLAMPED);
    }

    public void adjustClamped() {
        robot.airplaneLauncherAdjustServo.setPosition(RobotSettings.LAUNCHER_ADJUST_POSITION_CLAMPED);
    }

}
