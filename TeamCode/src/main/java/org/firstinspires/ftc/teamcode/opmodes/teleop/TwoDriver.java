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
    private boolean overrideLeft= false;
    private int overrideLeftCounter = 0;
    private double startingTimeLeft = 0;
    private boolean overrideRight = false;
    private int overrideRightCounter = 0;
    private double startingTimeRight = 0;


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
                if (currentGamepad2.left_bumper) {
                    overrideLeft = true;
                }else {
                    overrideLeft = false;
                }

                if (currentGamepad2.right_bumper) {
                    overrideRight = true;
                }else {
                    overrideRight = false;
                }

                if (overrideLeft) {
                    overrideLeftCounter++;
                    if (overrideLeftCounter == 1) {
                        startingTimeLeft = runtime.seconds();
                    }
                    double difference = runtime.seconds() - startingTimeLeft;
                    if (difference % 0.05 == 0) { // 20 encoder ticks change per second
                        robot.armSubsystem.encoderPosition++;
                    }
                }else {
                    overrideLeftCounter = 0;
                }
                
                if (overrideRight) {
                    overrideRightCounter++;
                    if (overrideRightCounter == 1) {
                        startingTimeRight = runtime.seconds();
                    }
                    double difference = runtime.seconds() - startingTimeRight;
                    if (difference % 0.05 == 0) {
                        robot.armSubsystem.encoderPosition--;
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

            }
        }

    }
}
