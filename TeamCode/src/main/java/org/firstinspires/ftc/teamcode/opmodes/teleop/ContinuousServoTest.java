package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Config
@TeleOp (name = "Continuous Servo Test", group = "Testing")
public class ContinuousServoTest extends LinearOpMode {

    private boolean launchPlane = false;
    private int counter = 0;

    public static double reverse = 0;
    public static double forward = 1.0;

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
                    counter++;
                }

                if (counter % 3 == 0) {
                    robot.airplaneLauncherServo.setPosition(reverse);
                }else if (counter % 3 == 1) {
                    robot.airplaneLauncherServo.setPosition(0.5);
                }else if (counter % 3 == 2) {
                    robot.airplaneLauncherServo.setPosition(forward);
                }

                robot.launcher.addTelemetry();
                telemetry.update();

            }
        }
        robot.stop();
    }

}
