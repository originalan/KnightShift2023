package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

/**
 * AirplaneLauncherTest is a test Teleop mode that is used purely to test the airplane launcher.
 */
@TeleOp (name = "Airplane Launcher Test", group = "Testing")
public class AirplaneLauncherTest extends LinearOpMode {

    private boolean launchPlane = false;

    @Override
    public void runOpMode() throws InterruptedException {

        JVBoysSoccerRobot robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

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
