package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.PIDControl;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.PurplePixel;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@TeleOp(name = "OneDriverTest", group = "Testing")

public class OneDriverTest extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;

    private boolean intakeOn = false;
    private boolean intakeReverse = false;
    private boolean launcherFired = false;

    private boolean moveRigServo = false;
    private boolean undoRig = true;
    private boolean a = true;
    private boolean deliverPurplePixel = false;
    private boolean switchDriveControls = false;
    private double timeElapsedRigging = 0;
    private int i = 1;
    private int clawCounter = 1;


    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        double reversed = 1.0;

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        // WAIT FOR THIS TELEMETRY MESSAGE BEFORE PRESSING START because IMU takes a while to be initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        while (opModeInInit()) {
            robot.rig.noHang();
            robot.purplePixelServo.setPosition(1);
            robot.launcher.notYet();
        }


        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Records joystick values
//                double axial = -1 * gamepad1.left_stick_y * reversed; // pushing stick forward gives negative value
//                double lateral = gamepad1.left_stick_x * reversed;
//                double yaw = gamepad1.right_stick_x * reversed;

                double axialIMU = gamepad1.left_stick_x;
                double lateralIMU = -1 * gamepad1.left_stick_y;
                double yawIMU = gamepad1.right_stick_x;

                // Prem said this is good cuz you can easily see what is wrong if robot strafes off
                double axialIMU2 = gamepad1.left_stick_x;
                double lateralIMU2 = -1 * gamepad1.right_stick_y;
                double yawIMU2 = (gamepad1.right_trigger - gamepad1.left_trigger);

                // Moves drivetrain on a field orientated drive and updates telemetry with wheel powers
//                if (switchDriveControls) {
//                    robot.drivetrain.goXYRIMU(axialIMU2, lateralIMU2, yawIMU2);
//                }else {
//                    robot.drivetrain.goXYRIMU(axialIMU, lateralIMU, yawIMU);
//                }

                // PID control that adjusts for any irl inconsistencies with motor velocity
                // drivetrain.checkAndAdjustMotors();

                // Show elapsed game time
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                robot.addTelemetry();

                // For debugging
//                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
//                    switchDriveControls = !switchDriveControls;
//                }

                if (Math.abs(currentGamepad1.left_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.ON;
                }
                else if (Math.abs(currentGamepad1.right_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.REVERSE;
                }
                else {
                    robot.intake.intakeState = Intake.IntakeState.OFF;
                }

//                if (currentGamepad1.right_bumper || currentGamepad1.left_bumper) {
//                    robot.rightRigMotor.setPower(0.4 * reversed);
//                    robot.leftRigMotor.setPower(-0.4 * reversed);
//                }else {
//                    robot.rightRigMotor.setPower(0);
//                    robot.leftRigMotor.setPower(0);
//                }

                if (currentGamepad1.a && !previousGamepad1.a) {
                    deliverPurplePixel = !deliverPurplePixel;
                }

//                if (currentGamepad1.b && !previousGamepad1.b) {
//                    robot.drivetrain.resetInitYaw();
//                }
//                if (currentGamepad1.x && !previousGamepad1.x) {
//                    moveRigServo = true;
//                    timeElapsedRigging = getRuntime();
//                    undoRig = !undoRig;
//                    i = 1;
//                }

                if (currentGamepad1.y && !previousGamepad1.y) {
                    clawCounter++;
                    if (clawCounter % 5 == 3) {
                        robot.linearSlideMotor.setTargetPosition(RobotSettings.OUTTAKE_MOTOR_ENCODER_POSITION);
                        robot.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (clawCounter % 5 == 0) {
                        robot.linearSlideMotor.setTargetPosition(0);
                        robot.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                }

                if (clawCounter % 5 == 1) {
                    robot.slide.slideState = LinearSlide.SlideState.OFF;
                }
                else if (clawCounter % 5 == 2) {
                    robot.slide.slideState = LinearSlide.SlideState.HOLDING_PIXEL;
                }
                else if (clawCounter % 5 == 3) {
                    robot.slide.slideState = LinearSlide.SlideState.BRING_ARM_IN_PLACE;
                }
                else if (clawCounter % 5 == 4) {
                    robot.slide.slideState = LinearSlide.SlideState.RELEASE_PIXEL;
                }
                else if (clawCounter % 5 == 0) {
                    robot.slide.slideState = LinearSlide.SlideState.BRING_ARM_BACK;
                    if (!robot.linearSlideMotor.isBusy()) {
                        robot.slide.slideState = LinearSlide.SlideState.OFF;
                        clawCounter++;
                    }
                }

                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    launcherFired = !launcherFired;
                }

                if (launcherFired) {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                }else {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.OFF;
                }

//                telemetry.addData("I value: ", i);

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
                        }
                    }else {
                        if (delta < incrementTime * i && delta >= incrementTime * (i-1) && i < (0.5 / incrementServo)) {
                            robot.rig.hang(incrementServo * i);
                            i++;
                        }
                        if (i >= (0.5 / incrementServo)) {
                            robot.rig.hang(0.5);
                        }
                    }
                }else {
                    robot.rig.noHang();
                }

                if (deliverPurplePixel) {
                    robot.purplePixel.purplePixelState = PurplePixel.PurplePixelState.HOLD;
                }else {
                    robot.purplePixel.purplePixelState = PurplePixel.PurplePixelState.DROP;
                }

                // Update all subsystems (if applicable since drivetrain needs no update)
                robot.update();
                telemetry.addData("slide state", robot.launcher.launcherState);
                telemetry.addData("claw counter", clawCounter);
                telemetry.addData("claw counter % 5", clawCounter % 5);
                telemetry.update();
            }
        }
//        robot.stop();
    }

}
