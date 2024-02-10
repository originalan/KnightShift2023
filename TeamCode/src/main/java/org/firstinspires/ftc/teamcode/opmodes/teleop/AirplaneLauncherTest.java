package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

/**
 * AirplaneLauncherTest is a test Teleop mode that is used purely to test the airplane launcher.
 */
@TeleOp (name = "Airplane Launcher Test", group = "Testing")
public class AirplaneLauncherTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private boolean launchPlane = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Press D-Pad Down once to fire, twice to go back");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    launchPlane = !launchPlane;
                }

                if (launchPlane) {
                    robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                }else {
                    robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.AT_REST;
                }

                robot.launcherSubsystem.addTelemetry();
                robot.launcherSubsystem.update();
                telemetry.update();

            }
        }

    }

}
