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

    private ElapsedTime launcherTimer = new ElapsedTime();
    public int counter = 0;

    public enum LauncherState {
        SETUP,
        AT_REST,
        ZONE_ONE_OR_BUST,
        NOTHING
    }
    public LauncherState launcherState = LauncherState.NOTHING;

    private LauncherState previousState = LauncherState.NOTHING;

    public AirplaneLauncher(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        launcherTimer.reset();
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.LAUNCHER) {
//            telemetry.addLine("Airplane Launcher");
//            telemetry.addData("   Launcher Fire Servo Current Position", "%4.2f", robot.launcherFireServo.getPosition());
//            telemetry.addData("   Launcher Adjust Servo Current Position
//            ", "%4.2f", robot.launcherAdjustServo.getPosition());
//            telemetry.addData("    Times states switched", counter);
        }
    }

    @Override
    public void update() {

//        if (previousState != launcherState) {
//            counter++;
//        }
//
//        switch (launcherState) {
//            case AT_REST:
//                restFireServo();
//                clampAdjustServo();
//                break;
//            case ZONE_ONE_OR_BUST:
//                unclampAdjustServo();
//                releaseFireServo();
//                break;
//            case SETUP:
//                restFireServo();
//                unclampAdjustServo();
//                break;
//            case NOTHING:
//                break;
//        }
//        previousState = launcherState;
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
