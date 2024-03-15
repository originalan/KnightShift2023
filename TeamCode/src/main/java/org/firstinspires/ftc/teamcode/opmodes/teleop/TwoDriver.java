package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Config
@TeleOp (name = "TwoDriver", group = "Final")
public class TwoDriver extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private double rigWaitTime = 0;
    private double doubleCheckWaitTime = 0;
    private double delayTime = 0;
    private double launchTime = 0;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private Gamepad currentGamepad2;
    private Gamepad previousGamepad2;

    // AIRPLANE LAUNCHER
    private boolean launcherFired = false;
    private boolean fireAirplane = false;

    private boolean switchDriveControls = false;
    private boolean useDSensors = false;

    public static boolean leftClosed = true, rightClosed = true;
    public static boolean orientHelp = false;
    private double previousX = 5, previousY = 5, previousR = 5;
    private boolean isRed = true;

    private enum IntakeControlsState {
        INTAKING, // claw on the floor and open for pixels
        INTAKING_AUTO,
        CLOSED, // claw up and closed, holding pixels
//        DELAY, // arm is moving, wait 0.25 seconds before moving claw subsystems
        GO_BACK_DOWN, // from any other arm position, going back down to closed position
        DOUBLE_CHECK, // reset the arm encoder position again after 0.5 seconds of GO_BACK_DOWN
        DROP_POS,
        RESET, // reset arm encoder and brings claw to closed position
        NOTHING, // idk yet
    }
    private enum RiggingControlsState {
        DOWN_WAIT,
        DOWN,
        UP,
        HANGING,
        NOTHING
    }
    private enum LauncherControlsState {
        REST,
        READY,
        FIRE,
        OFF
    }
    private IntakeControlsState intakeState = IntakeControlsState.CLOSED;
    private RiggingControlsState hangState = RiggingControlsState.DOWN;
    private LauncherControlsState launchState = LauncherControlsState.REST;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Make sure both gamepads are connected");
        telemetry.addLine("Gamepad2: x, y, a, b = arm preset positions");
        telemetry.addLine("    right/left bumper in closed state = reset arm encoder");
        telemetry.addLine("    right/left bumpers elsewhere = open / close claw");
        telemetry.addLine("    dpad down = intake auto (using dsensors)");
        telemetry.addLine("    dpad up = intake manual");
        telemetry.addLine("Gamepad1: drivetrain movement using joysticks");
        telemetry.addLine("    x to rig, left/right bumpers to move string");
        telemetry.addLine("    b to switch field-oriented drive to robot-oriented drive");
        telemetry.addLine("    a to help move straight");
        telemetry.addLine("    y to use dsensors");
        telemetry.addLine("    dpad down to fire airplane");
        telemetry.addLine("    dpad up to reset yaw for field-oriented drive");
        telemetry.addLine("    left/right triggers to slow down drivetrain by factor of 3.0");
        telemetry.update();

        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
        robot.armSubsystem.pivotState = Arm.PivotState.REST;
        robot.armSubsystem.armState = Arm.ArmState.NOTHING;

        while (opModeInInit()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.x && !previousGamepad1.x) {
                isRed = !isRed;
            }
            telemetry.addData("WHICH SIDE? ", isRed ? "RED" : "BLUE");
            telemetry.update();
        }

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();
                // Gamepad1 = driving, rigging, airplane launcher
                // Gamepad2 = all arm control, failsafes/override commands

                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                // Failsafe field-oriented view
                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    robot.drivetrainSubsystem.resetInitYaw();
                }

                if (robot.armSubsystem.armState == Arm.ArmState.MOTION_PROFILE) {

                    drivetrainControls();
                    intakeControls();

//                    robot.addTelemetry();
                    robot.armSubsystem.addTelemetry();
                    telemetry.addData("Intake State", intakeState);
                    telemetry.update();
                    robot.armSubsystem.update();
                    robot.clawSubsystem.update();

                }else {

                    drivetrainControls();

                    rigControls();

                    launcherControls();

                    intakeControls();

                    robot.addTelemetry();
                    telemetry.addData("Intake State", intakeState);
                    telemetry.update();
                    robot.update();
                }

            }
        }

    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        if (currentGamepad1.b && !previousGamepad1.b) {
            switchDriveControls = !switchDriveControls;
        }

        if (currentGamepad1.a && !previousGamepad1.a) {
            orientHelp = !orientHelp;
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            useDSensors = !useDSensors;
        }

//        if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
//            robot.drivetrainSubsystem.dSensorCheck();
//        }

        if (useDSensors) {
            robot.drivetrainSubsystem.dSensorCheck();
        }


        if (currentGamepad1.right_trigger > 0.01 || currentGamepad1.left_trigger > 0.01) {
            x /= 3;
            y /= 3;
            r /= 3;
        }

        // attempting to save motor calls == faster frequency of command calls
        if ( !(previousX == x && previousY == y && previousR == r) ) {
            robot.drivetrainSubsystem.moveXYR(x, y, r, !switchDriveControls);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }

    public void rigControls() {
        switch (hangState) {
            case UP:
                if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                    hangState = RiggingControlsState.HANGING;
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    rigWaitTime = runtime.seconds();
                    robot.rigRightServo.setPwmEnable();
                    robot.rigLeftServo.setPwmEnable();
                    hangState = RiggingControlsState.DOWN_WAIT;
                }
                break;
            case DOWN:
                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.rigRightServo.setPwmEnable();
                    robot.rigLeftServo.setPwmEnable();
                    hangState = RiggingControlsState.UP;
                    robot.riggingSubsystem.riggingState = Rigging.RiggingState.RIG;
                }
                break;
            case DOWN_WAIT:
                robot.riggingSubsystem.riggingState = Rigging.RiggingState.NO_RIG;
                if (runtime.seconds() - rigWaitTime > 1.0) {
                    robot.riggingSubsystem.riggingState = Rigging.RiggingState.NOTHING;
                    hangState = RiggingControlsState.DOWN;
                    robot.rigRightServo.setPwmDisable();
                    robot.rigLeftServo.setPwmDisable();
                }
                break;
            case HANGING:
                robot.riggingSubsystem.riggingState = Rigging.RiggingState.NOTHING;
                intakeState = IntakeControlsState.CLOSED;
                robot.rigRightServo.setPwmDisable();
                robot.rigLeftServo.setPwmDisable();
                if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                    robot.riggingSubsystem.setRiggingMotors(RobotSettings.RIGGING_MOTOR_SPEED);
                }else {
                    robot.riggingSubsystem.setRiggingMotors(0);
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    rigWaitTime = runtime.seconds();
                    robot.rigRightServo.setPwmEnable();
                    robot.rigLeftServo.setPwmEnable();
                    hangState = RiggingControlsState.DOWN_WAIT;
                }
                break;
            case NOTHING:
                break;
        }
    }

    public void launcherControls() {
        switch (launchState) {
            case REST:
                robot.testServo.setPwmDisable();
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                    launchState = LauncherControlsState.READY;
//                    launchTime = runtime.seconds();
//                    double angle = robot.drivetrainSubsystem.initYaw - robot.drivetrainSubsystem.lastAngle.firstAngle;
//                    angle %= 360;
//                    double a;
//                    if (isRed) {
//                        a = 90 - angle - 5;
//                    }else {
//                        a = -1 * (90 - angle - 5);
//                    }
//                    TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(angle)))
//                            .turn( Math.toRadians(a) )
//                            .build();
//                    drive.followTrajectorySequenceAsync(traj);
                    launchState = LauncherControlsState.FIRE;
                }
                break;
            case READY:
                if (!drive.isBusy()) {
                    launchState = LauncherControlsState.FIRE;
                }
                break;
            case OFF:
                robot.testServo.setPwmDisable();
                break;
            case FIRE:
                robot.testServo.setPwmEnable();
                robot.testServo.setPosition(0.7);
                if (runtime.seconds() - launchTime > 1.0) {
                    launchState = LauncherControlsState.OFF;
                }
                break;
        }
    }

    public void clawSidePieceControls(boolean reversed) {
        if (reversed) {
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                rightClosed = !rightClosed;
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                leftClosed = !leftClosed;
            }
        }else {
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                leftClosed = !leftClosed;
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                rightClosed = !rightClosed;
            }
        }

        if (leftClosed && rightClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
        }
        if (leftClosed && !rightClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
        }
        if (rightClosed && !leftClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
        }
        if (!rightClosed && !leftClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
        }
    }

    public void intakeControls() {

        switch (intakeState) {
            case CLOSED:
//                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
//                robot.armSubsystem.pivotState = Arm.PivotState.REST;

                // reset button
                if (currentGamepad2.right_bumper && currentGamepad2.left_bumper) {
                    intakeState = IntakeControlsState.RESET;
                }

                if (currentGamepad2.y && !previousGamepad2.y) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfile(ArmSettings.position1);
                    intakeState = IntakeControlsState.DROP_POS;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfile(ArmSettings.position2);
                    intakeState = IntakeControlsState.DROP_POS;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfile(ArmSettings.position3);
                    intakeState = IntakeControlsState.DROP_POS;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    leftClosed = true;
                    rightClosed = true;
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfile(ArmSettings.position4);
                    intakeState = IntakeControlsState.DROP_POS;
                }

                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up /*  && withinRange(BulkReading.pArmLeftMotor, -5, 5) */ ) {
                    leftClosed = false; // left claw open
                    rightClosed = false; // right claw open
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    intakeState = IntakeControlsState.INTAKING;
                }
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down /*  && withinRange(BulkReading.pArmLeftMotor, -5, 5) */ ) {
                    leftClosed = false; // left claw open
                    rightClosed = false; // right claw open
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    intakeState = IntakeControlsState.INTAKING_AUTO;
                }
                break;
            case GO_BACK_DOWN:
                // robot arm is moving back down to intaking / closed position
                if (robot.armSubsystem.getMP().getInstantPosition() < 375) {
                    robot.armSubsystem.pivotState = Arm.PivotState.REST;
                    robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    robot.armSubsystem.setArmPower(0);
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;

                    doubleCheckWaitTime = runtime.seconds();
                    intakeState = IntakeControlsState.DOUBLE_CHECK;
                }
                break;
            case DOUBLE_CHECK:
                // let arm settle for 0.25 seconds
                if (runtime.seconds() - doubleCheckWaitTime > 0.4) {
                    intakeState = IntakeControlsState.RESET;
                }
                break;
            case INTAKING:
                robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                clawSidePieceControls(true);

                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    intakeState = IntakeControlsState.RESET;
                }
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    intakeState = IntakeControlsState.INTAKING_AUTO;
                }
                break;
            case INTAKING_AUTO:
                robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                robot.clawSubsystem.clawState = Claw.ClawState.AUTO_DETECT;

                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    intakeState = IntakeControlsState.INTAKING;
                }
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    intakeState = IntakeControlsState.RESET;
                }
                break;
            case DROP_POS:
                clawSidePieceControls(false);

                if (currentGamepad2.y && !previousGamepad2.y) {
                    robot.armSubsystem.setMotionProfile(ArmSettings.position1);
                    robot.armSubsystem.pivotState = Arm.PivotState.REST;
                }
                if (currentGamepad2.a && !previousGamepad2.a) {
                    robot.armSubsystem.setMotionProfile(ArmSettings.position2);
                    robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
                }
                if (currentGamepad2.b && !previousGamepad2.b) {
                    robot.armSubsystem.setMotionProfile(ArmSettings.position3);
                    robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
                }
                if (currentGamepad2.x && !previousGamepad2.x) {
                    robot.armSubsystem.setMotionProfile(ArmSettings.position4);
                    robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
                }

                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    robot.armSubsystem.setMotionProfile(ArmSettings.positionBottom);
                    delayTime = runtime.seconds();
                    intakeState = IntakeControlsState.GO_BACK_DOWN;
                }
                break;
            case RESET:
                robot.armSubsystem.setArmPower(0);
                robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.armSubsystem.pivotState = Arm.PivotState.REST;
                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                robot.armSubsystem.armState = Arm.ArmState.NOTHING;

                intakeState = IntakeControlsState.CLOSED;
                break;
            case NOTHING:
                break;
        }

    }
    public boolean withinRange(double value, double bottom, double top) {

        return value >= bottom && value <= top;

    }

}
