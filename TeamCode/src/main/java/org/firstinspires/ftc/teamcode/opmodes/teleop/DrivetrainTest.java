package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@TeleOp (name = "Drivetrain Test", group = "Testing")
public class DrivetrainTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private int counter = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Press X multiple times to cycle through different drivetrain codes");
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                double x = gamepad1.left_stick_x * 1.05;
                double y = gamepad1.left_stick_y * -1;
                double r = gamepad1.right_stick_x;

                if (currentGamepad1.x && !previousGamepad1.x) {
                    counter++;
                    if (counter > 4) {
                        counter = 1;
                    }
                }

                switch (counter) {
                    case 1:
                        robot.drivetrainSubsystem.goXYR(y, x, r);
                        telemetry.addLine("goXYR() method in use");
                        break;
                    case 2:
                        robot.drivetrainSubsystem.goXYRIMU(x, y, r);
                        telemetry.addLine("goXYRIMU() method in use");
                        break;
                    case 3:
                        robot.drivetrainSubsystem.moveXYR(x, y, r, true);
                        telemetry.addLine("moveXYR() with IMU method in use");
                        break;
                    case 4:
                        robot.drivetrainSubsystem.moveXYR(x, y, r, false);
                        telemetry.addLine("moveXYR() without IMU method in use");
                        break;
                }

                robot.drivetrainSubsystem.addTelemetry();
                robot.update();
                telemetry.update();

            }
        }

    }
}
