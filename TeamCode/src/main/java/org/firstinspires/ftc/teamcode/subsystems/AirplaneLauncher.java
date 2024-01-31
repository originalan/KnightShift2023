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
        SETUP,
        AT_REST,
        ZONE_ONE_OR_BUST,
        NOTHING
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
        telemetry.addData("   Launcher Fire Servo Current Position", "%4.2f", robot.launcherFireServo.getPosition());
        telemetry.addData("   Launcher Adjust Servo Current Position", "%4.2f", robot.launcherAdjustServo.getPosition());
    }

    @Override
    public void update() {
        switch (launcherState) {
            case AT_REST:
                restFireServo();
                clampAdjustServo();
                break;
            case ZONE_ONE_OR_BUST:
                if (timerCounter == 1) {
                    timer.reset();
                    timerCounter++;
                }
                currentTime = timer.seconds();
                unclampAdjustServo();
                releaseFireServo();
                if (currentTime > 2.0) { // unclamp, then wait 2 seconds before firing airplane

                }
                break;
            case SETUP:
                restFireServo();
                unclampAdjustServo();
                break;
            case NOTHING:
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void releaseFireServo() {
        robot.launcherFireServo.setPosition(RobotSettings.LAUNCHER_FIRE_POSITION_FIRE);
    }

    public void restFireServo() {
        robot.launcherFireServo.setPosition(RobotSettings.LAUNCHER_FIRE_POSITION_REST);
    }

    public void unclampAdjustServo() {
        robot.launcherAdjustServo.setPosition(RobotSettings.LAUNCHER_ADJUST_POSITION_UNCLAMPED);
    }

    public void clampAdjustServo() {
        robot.launcherAdjustServo.setPosition(RobotSettings.LAUNCHER_ADJUST_POSITION_CLAMPED);
    }

}
