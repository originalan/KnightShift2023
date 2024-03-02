package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.util.ArmSettings.ENCODER_TICKS_PER_SECOND;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.ArmSettings;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@Config
@TeleOp (name = "TwoDriver (MP)", group = "Final")
public class TwoDriver extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime1 = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    private ElapsedTime resetZeroTimer = new ElapsedTime();
    private int counter = 1;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    // RIGGING
    private boolean isRigging = false;
    private boolean rigStringMove = false;
    private double rigTime = 0;
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

    private boolean left = true, right = true;


    @Override
    public void runOpMode() {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Make sure both gamepads are connected");
        telemetry.addLine("Gamepad2: x, y, a, b = arm preset positions");
        telemetry.addLine("    right/left triggers = override arm position");
        telemetry.addLine("    right/left bumpers = open / close claw");
        telemetry.addLine("    right/left dpads = pivot up, pivot ground");
        telemetry.addLine("    dpad down = auto orient pivot claw");
        telemetry.addLine("    dpad up to switch field-oriented drive to robot-oriented drive");
        telemetry.addLine("Gamepad1: drivetrain movement using joysticks");
        telemetry.addLine("    x to rig, left/right bumpers to move string");
        telemetry.addLine("    dpad down to fire airplane");
        telemetry.addLine("    dpad up to reset yaw for field-oriented drive");
        telemetry.addLine("    left/right triggers to use distance sensor backdrop thing");
        telemetry.update();

        waitForStart();

        runtime.reset();
        rigTime = runtime.seconds();
        robot.armSubsystem.armState = Arm.ArmState.NOTHING;

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

                drivetrainControls();

                /*
                =================RIGGING CONTROLS==============
                */

                riggingControls();

                /*
                =================AIRPLANE CONTROLS==============
                */

                launcherControls();

                /*
                =================ARM CONTROLS==============
                */

//                armControls();
                armMotionProfileControls();

                /*
                =================CLAW CONTROLS==============
                */

                pivotClawControls();
                clawSidePieceControls();

                /*
                =================FAILSAFE FIELD-ORIENTED VIEW CONTROLS==============
                */

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    robot.drivetrainSubsystem.resetInitYaw();
                }

                robot.addTelemetry();
                telemetry.update();
                robot.update();

            }
        }

    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        robot.drivetrainSubsystem.factor = 1.0;

        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            switchDriveControls = !switchDriveControls;
        }

        if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            robot.drivetrainSubsystem.dSensorCheck();
        }

        robot.drivetrainSubsystem.moveXYR(x, y, r, !switchDriveControls);
    }

    public void riggingControls() {
        if (currentGamepad1.x && !previousGamepad1.x) {
            isRigging = !isRigging;
            rigTime = runtime.seconds();
        }

        if (!rigStringMove && isRigging) {
            robot.riggingSubsystem.hang();
            robot.rigRightServo.getController().pwmEnable();
            robot.rigLeftServo.getController().pwmEnable();
        }else if (!isRigging) {
            if (runtime.seconds() - rigTime < 1.5) {
                robot.rigRightServo.getController().pwmEnable();
                robot.rigLeftServo.getController().pwmEnable();
                robot.riggingSubsystem.noHang();
            }else {
                robot.rigRightServo.getController().pwmDisable();
                robot.rigLeftServo.getController().pwmDisable();
            }
        }
        if (isRigging && rigStringMove) {
            robot.rigRightServo.getController().pwmDisable();
            robot.rigLeftServo.getController().pwmDisable();
        }

        // If arms are up and motor power buttons are pressed...
        if (isRigging && (currentGamepad1.left_bumper || currentGamepad1.right_bumper)) {
            rigStringMove = true;
            robot.rigRightMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
            robot.rigLeftMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
        }else {
            robot.rigRightMotor.setPower(0);
            robot.rigLeftMotor.setPower(0);
        }
    }

    public void launcherControls() {
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            launcherFired = !launcherFired;
        }

        if (launcherFired) {
//                    robot.launcherSubsystem.counter++;
            robot.launcherSubsystem.releaseFireServo();
        }else {
//                    robot.launcherSubsystem.counter++;
            robot.launcherSubsystem.restFireServo();
        }
    }

    public void armControls() {
        robot.armSubsystem.armState = Arm.ArmState.NOTHING;
        if (currentGamepad1.y && !previousGamepad1.y) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.position1;
        }
        if (currentGamepad1.a && !previousGamepad1.a) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.position2;
        }
        if (currentGamepad1.b && !previousGamepad1.b) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.position3;
        }
        if (currentGamepad1.x && !previousGamepad1.x) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.positionBottom;
            resetZeroTimer.reset();
            counter = 2;
        }

        if (resetZeroTimer.seconds() > 4.0 && counter == 2) {
            robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            counter = 1;
        }

        // Manual "override" of PIDF control of arm
        overrideLeft = currentGamepad2.left_trigger > 0.01;

        overrideRight = currentGamepad2.right_trigger > 0.01;

        if (overrideLeft && !overrideRight) {
            overrideLeftCounter++;
            if (overrideLeftCounter == 1) {
                startingTimeLeft = runtime1.seconds();
            }
            double difference = runtime1.seconds() - startingTimeLeft;
            if (difference > 1.0 / ENCODER_TICKS_PER_SECOND) { // 20 encoder ticks change per second
                robot.armSubsystem.encoderGoalPosition++;
                runtime1.reset();
            }
        }else {
            overrideLeftCounter = 0;
        }


        if (overrideRight && !overrideLeft) {
            overrideRightCounter++;
            if (overrideRightCounter == 1) {
                startingTimeRight = runtime2.seconds();
            }
            double difference2 = runtime2.seconds() - startingTimeRight;
            if (difference2 > 1.0 / ENCODER_TICKS_PER_SECOND) { // 20 encoder ticks change per second
                robot.armSubsystem.encoderGoalPosition--;
                runtime2.reset();
            }
        }else {
            overrideRightCounter = 0;
        }

    }

    public void armMotionProfileControls() {
        if (currentGamepad2.y && !previousGamepad2.y) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.position1;
            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
            robot.armSubsystem.setMotionProfile();
        }
        if (currentGamepad2.a && !previousGamepad2.a) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.position2;
            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
            robot.armSubsystem.setMotionProfile();
        }
        if (currentGamepad2.b && !previousGamepad2.b) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.position3;
            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
            robot.armSubsystem.setMotionProfile();
        }
        if (currentGamepad2.x && !previousGamepad2.x) {
            robot.armSubsystem.encoderGoalPosition = ArmSettings.positionBottom;
            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
            robot.armSubsystem.setMotionProfile();
            resetZeroTimer.reset();
            counter = 2;
        }

        // If timer is 4 seconds, run once (done with the counter variable), and the goal position is still 0
        if (resetZeroTimer.seconds() > 4.0 && counter == 2 && robot.armSubsystem.encoderGoalPosition == ArmSettings.positionBottom) {
            robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            counter = 1;
        }

        // Manual "override" of PIDF control of arm
//        overrideLeft = currentGamepad2.left_trigger > 0.01 && !(previousGamepad2.left_trigger > 0.01);
//
//        overrideRight = currentGamepad2.right_trigger > 0.01 && !(previousGamepad2.right_trigger > 0.01);;
//
//        if (overrideLeft || overrideRight) {
//            robot.armSubsystem.armState = Arm.ArmState.NOTHING;
//            robot.armSubsystem.setArmEncoderPosition(robot.armSubsystem.encoderGoalPosition);
//        }
//
//        // Increase encoder ticks
//        if (overrideRight && !overrideLeft) {
//            startingTimeRight++;
//            if (startingTimeRight == 1) {
//                overrideRightCounter = robot.armSubsystem.encoderGoalPosition;
//                runtime1.reset();
//            }
//            robot.armSubsystem.encoderGoalPosition = overrideRightCounter + (int)(runtime1.seconds() * 20);
//        }else {
//            startingTimeRight = 0;
//        }
//
//        // Decrease encoder ticks
//        if (overrideLeft && !overrideRight) {
//            startingTimeLeft++;
//            if (startingTimeLeft == 1) {
//                overrideLeftCounter = robot.armSubsystem.encoderGoalPosition;
//                runtime2.reset();
//            }
//            robot.armSubsystem.encoderGoalPosition = overrideLeftCounter - (int)(runtime2.seconds() * 20);
//        }else {
//            overrideLeftCounter = 0;
//        }

        if (overrideLeft) {
            robot.armSubsystem.encoderGoalPosition += 10;
        }
        if (overrideRight) {
            robot.armSubsystem.encoderGoalPosition -= 10;
        }

    }

    public void pivotClawControls() {
        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
        }
        if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            robot.armSubsystem.pivotState = Arm.PivotState.REST;
        }
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
        }
    }

    public void clawSidePieceControls() {
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            left = !left;
        }
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            right = !right;
        }

        if (left && right) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
        }
        if (left && !right) {
            robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
        }
        if (right && !left) {
            robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
        }
        if (!right && !left) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
        }
    }

}
