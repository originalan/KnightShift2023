package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@TeleOp (name = "Airplane Launcher Test", group = "testing")
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

                if (currentGamepad1.b && !previousGamepad1.b) {
                    launchPlane = !launchPlane;
                }

                if (launchPlane) {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                }else {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.OFF;
                }

                robot.launcher.addTelemetry();
                telemetry.update();

            }
        }
        robot.stop();
    }

}
