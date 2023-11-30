package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@TeleOp (name = "4 Kidz", group = "KIDS")
public class ForKids extends LinearOpMode {

    private boolean switchDriveControls = false;

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        JVBoysSoccerRobot robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addLine("Press the START button in order to drive the robot!");
        telemetry.update();

        waitForStart();

        if( opModeIsActive() ) {
            while (opModeIsActive()) {

                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Records joystick values
                double axial = -1 * gamepad1.left_stick_y; // pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double axialIMU = gamepad1.left_stick_x;
                double lateralIMU = -1 * gamepad1.left_stick_y;
                double yawIMU = gamepad1.right_stick_x;

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    switchDriveControls = !switchDriveControls;
                }

                robot.drivetrain.goXYR(axial, lateral, yaw);

                telemetry.addLine("Use the joysticks to move the robot!");
                telemetry.update();

            }
        }

    }

}
