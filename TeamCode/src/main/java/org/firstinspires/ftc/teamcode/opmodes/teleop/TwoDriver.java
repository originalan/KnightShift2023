package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@Config
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

    // ARMS (override move arm left or right)
    private boolean overrideLeft= false;
    private int overrideLeftCounter = 0;
    private double startingTimeLeft = 0;
    private boolean overrideRight = false;
    private int overrideRightCounter = 0;
    private double startingTimeRight = 0;


    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        waitForStart();

        runtime.reset();

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
                double x = gamepad1.left_stick_x * 1.05;
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

                if (!rigStringMove && isRigging) {
                    robot.riggingSubsystem.hang();
                    robot.rigRightServo.getController().pwmEnable();
                    robot.rigLeftServo.getController().pwmEnable();
                }else {
                    robot.rigRightServo.getController().pwmDisable();
                    robot.rigLeftServo.getController().pwmDisable();
                }

                if (isRigging && (currentGamepad1.dpad_right || currentGamepad1.dpad_left)) {
                    rigStringMove = true;
                    robot.rigRightMotor.setPower(-1 * RobotSettings.RIGGING_MOTOR_SPEED);
                    robot.rigLeftMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                }else {
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
                    robot.launcherSubsystem.counter++;
                    robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                }else {
                    robot.launcherSubsystem.counter++;
                    robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.AT_REST;
                }

                /*
                =================ARM CONTROLS==============
                */
                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.encoderPosition = Arm.position1;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    robot.armSubsystem.encoderPosition = Arm.position2;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.armSubsystem.encoderPosition = Arm.position3;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.encoderPosition = Arm.positionBottom;
                }

                // Manual "override" of PIDF control of arm
                overrideLeft = currentGamepad2.left_bumper;

                overrideRight = currentGamepad2.right_bumper;

                if (overrideLeft && !overrideRight) {
                    overrideLeftCounter++;
                    if (overrideLeftCounter == 1) {
                        startingTimeLeft = runtime.seconds();
                    }
                    double difference = runtime.seconds() - startingTimeLeft;
                    if (difference > 0.05) { // 20 encoder ticks change per second
                        robot.armSubsystem.encoderPosition++;
                        runtime.reset();
                    }
                }else {
                    overrideLeftCounter = 0;
                }


                if (overrideRight && !overrideLeft) {
                    overrideRightCounter++;
                    if (overrideRightCounter == 1) {
                        startingTimeRight = runtime.seconds();
                    }
                    double difference2 = runtime.seconds() - startingTimeRight;
                    if (difference2 > 0.05) {
                        robot.armSubsystem.encoderPosition--;
                        runtime.reset();
                    }
                }else {
                    overrideRightCounter = 0;
                }

                robot.armSubsystem.setArmEncoderPosition(robot.armSubsystem.encoderPosition);

                /*
                =================FAILSAFE FIELD-ORIENTED VIEW CONTROLS==============
                */
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    robot.drivetrainSubsystem.resetInitYaw();
                }

                robot.addTelemetry();
                telemetry.update();
                robot.update();

            }
        }

    }
}
