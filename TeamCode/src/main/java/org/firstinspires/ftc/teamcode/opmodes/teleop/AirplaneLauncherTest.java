package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

/**
 * AirplaneLauncherTest is a test Teleop mode that is used purely to test the airplane launcher.
 */
@TeleOp (name = "Airplane Launcher Test", group = "Testing")
public class AirplaneLauncherTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private boolean launchPlane = false;
    private double launchTime = 0;
    private enum LauncherControlsState {
        REST,
        FIRE,
        OFF
    }
    private LauncherControlsState launchState = LauncherControlsState.REST;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Press D-Pad Down once to fire, twice to go back");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                switch (launchState) {
                    case REST:
                        robot.testServo.getController().pwmDisable();
                        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                            launchState = LauncherControlsState.FIRE;
                            launchTime = runtime.seconds();
                        }
                        break;
                    case OFF:
                        robot.testServo.getController().pwmDisable();
                        break;
                    case FIRE:
                        robot.testServo.getController().pwmEnable();
                        robot.testServo.setPosition(0.7);
                        if (runtime.seconds() - launchTime > 1.0) {
                            launchState = LauncherControlsState.OFF;
                        }
                        break;
                }

                robot.launcherSubsystem.addTelemetry();
                robot.launcherSubsystem.update();
                telemetry.update();

            }
        }

    }

}
