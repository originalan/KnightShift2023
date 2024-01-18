package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * OneDriver is a Teleop mode that only requires one controller.
 * Manages joystick controls and telemetry updates.
 * It is NOT updated yet as of 12.08.2023. See OneDriverTest for latest updates
 */
@TeleOp(name = "OneDriver (OUTDATED)", group = "TeleOpmode")
public class OneDriver extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private boolean launcherFired = false;
    private boolean hooksInPlace = false;
    private boolean moveRigServo = false;
    private boolean undoRig = true;
    private double timeElapsedRigging = 0;
    private int i = 1;

    private boolean switchDriveControls = false;


    @Override
    public void runOpMode() {
        telemetry.addLine("WAIT FOR INITIALIZATION MESSAGE BEFORE PRESSING START");
        telemetry.update();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        // WAIT FOR THIS TELEMETRY MESSAGE BEFORE PRESSING START because IMU takes a while to be initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        while (opModeInInit()) {
            robot.rig.noHang();
            robot.launcher.launcherAtRest();
        }

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
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

                if (switchDriveControls) {
                    robot.drivetrain.goXYR(axial, lateral, yaw);
                }else {
                    robot.drivetrain.goXYRIMU(axialIMU, lateralIMU, yawIMU);
                }

                // Show elapsed game time
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                // Add all robot telemetry
                robot.addTelemetry();

                /*
                =================INTAKE CONTROLS==============
                */
                if (Math.abs(currentGamepad1.left_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.FORWARD;
                }
                else if (Math.abs(currentGamepad1.right_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.REVERSE;
                }
                else {
                    robot.intake.intakeState = Intake.IntakeState.OFF;
                }

                /*
                =================RIGGING CONTROLS==============
                */
                if (hooksInPlace && (currentGamepad1.right_bumper || currentGamepad1.left_bumper)) {
                    robot.rightRigMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                    robot.leftRigMotor.setPower(-1 * RobotSettings.RIGGING_MOTOR_SPEED);
                }else {
                    robot.rightRigMotor.setPower(0);
                    robot.leftRigMotor.setPower(0);
                }

                if (moveRigServo) {
                    double currentTime = getRuntime();
                    double delta = currentTime - timeElapsedRigging;
                    double incrementTime = 0.1; // in seconds
                    double incrementServo = 0.025;

                    if (undoRig) {
                        if (delta < incrementTime * i && delta >= incrementTime * (i-1) && i < (0.5 / incrementServo)) {
                            robot.rig.hang(0.5 - (incrementServo * i));
                            i++;
                        }
                        if (i >= (0.5 / incrementServo)) {
                            robot.rig.hang(0);
                            moveRigServo = false;
                            hooksInPlace = false;
                        }
                    }else {
                        if (delta < incrementTime * i && delta >= incrementTime * (i-1) && i < (0.5 / incrementServo)) {
                            robot.rig.hang(incrementServo * i);
                            i++;
                        }
                        if (i >= (0.5 / incrementServo)) {
                            robot.rig.hang(0.5);
                            hooksInPlace = true;
                        }
                    }
                }else {
                    robot.rig.noHang();
                }

                if (currentGamepad1.x && !previousGamepad1.x) {
                    moveRigServo = true;
                    timeElapsedRigging = getRuntime();
                    undoRig = !undoRig;
                    i = 1;
                }

                /*
                =================AIRPLANE CONTROLS==============
                */
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    launcherFired = !launcherFired;
                }

                if (launcherFired) {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                }else {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.AT_REST;
                }

                /*
                =================FAILSAFE FIELD-ORIENTED VIEW CONTROLS==============
                */
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.drivetrain.resetInitYaw();
                }

                // Update all subsystems (if applicable since drivetrain needs no update)
                robot.update();

                telemetry.update();
            }
        }

    }

}
