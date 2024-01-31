package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@TeleOp (name = "TwoDriver", group = "Final")
public class TwoDriver extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private ElapsedTime runtime = new ElapsedTime();

    // RIGGING
    private boolean isRigging = false;
    private boolean rigStringMove = false;
    // AIRPLANE LAUNCHER
    private boolean launcherFired = false;

    private boolean switchDriveControls = false;

    // ARMS
    private boolean atPos1 = false;
    private boolean atPos2 = false;
    private boolean atPos3 = false;
    private boolean atBottom = false;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("WAIT FOR INITIALIZATION MESSAGE BEFORE PRESSING START");
        telemetry.update();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Gamepad1 = driving, rigging, airplane launcher
                // Gamepad2 = all arm control, failsafes/override commands

                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                /*
                =================DRIVETRAIN CONTROLS==============
                 */
                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y * -1;
                double r = gamepad1.right_stick_x;

                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    switchDriveControls = !switchDriveControls;
                }

                robot.drivetrainSubsystem.moveXYR(x, y, r, !switchDriveControls);

                /*
                =================RIGGING CONTROLS==============
                */

                if (currentGamepad1.x && !previousGamepad1.x) {
                    isRigging = !isRigging;
                }

                if (!rigStringMove) {
                    if (isRigging) {
                        robot.rigRightServo.getController().pwmEnable();
                        robot.rigLeftServo.getController().pwmEnable();
                        robot.riggingSubsystem.hang();
                    }else {
                        robot.rigRightServo.getController().pwmDisable();
                        robot.rigLeftServo.getController().pwmDisable();
                    }
                }else {
                    robot.rigRightServo.getController().pwmDisable();
                    robot.rigLeftServo.getController().pwmDisable();
                }

                if (isRigging && (currentGamepad1.dpad_right || currentGamepad1.dpad_left)) {
                    rigStringMove = true;
                    robot.rigRightMotor.setPower(-1 * RobotSettings.RIGGING_MOTOR_SPEED);
                    robot.rigLeftMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                }
                else {
                    robot.rigRightMotor.setPower(0);
                    robot.rigLeftMotor.setPower(0);
                }

                /*
                =================AIRPLANE CONTROLS==============
                */
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    launcherFired = !launcherFired;
                }

                if (launcherFired) {
                    robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                    telemetry.addLine("LAUNCHER FIRED");
                }else {
                    robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.AT_REST;
                    telemetry.addLine("LAUNCHER AT REST");
                }

                /*
                =================ARM CONTROLS==============
                */
                if (currentGamepad2.y && !previousGamepad2.y) {
                    atPos1 = !atPos1;
                    atPos2 = false;
                    atPos3 = false;
                    atBottom = false;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    atPos1 = false;
                    atPos2 = false;
                    atPos3 = false;
                    atBottom = !atBottom;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    atPos1 = false;
                    atPos2 = !atPos2;
                    atPos3 = false;
                    atBottom = false;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    atPos1 = false;
                    atPos2 = false;
                    atPos3 = !atPos3;
                    atBottom = false;
                }

                if (atPos1) {
                    robot.armSubsystem.armState = Arm.ArmState.POS1;
                }
                if (atPos2) {
                    robot.armSubsystem.armState = Arm.ArmState.POS2;
                }
                if (atPos3) {
                    robot.armSubsystem.armState = Arm.ArmState.POS3;
                }
                if (atBottom) {
                    robot.armSubsystem.armState = Arm.ArmState.BOTTOM;
                }

                // Manual "override" of PIDF control of arm
                if (currentGamepad2.left_bumper) {
                    robot.armSubsystem.overridePowerForward = true;
                }else {
                    robot.armSubsystem.overridePowerForward = false;
                }

                if (currentGamepad2.right_bumper) {
                    robot.armSubsystem.overridePowerBackward = true;
                }else {
                    robot.armSubsystem.overridePowerBackward = false;
                }

                /*
                =================FAILSAFE FIELD-ORIENTED VIEW CONTROLS==============
                */
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    robot.drivetrainSubsystem.resetInitYaw();
                }

            }
        }

    }
}
