package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.util.BulkReading;

@TeleOp (name = "Outreach Opmode", group = "a")
public class Outreach extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private TrajectorySequence waitingOneHalfSeconds, waitingTime;

    private double rigWaitTime = 0;
    private double doubleCheckWaitTime = 0;
    private double delayTime = 0;
    private double launchTime = 0;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    public static boolean leftClosed = true, rightClosed = true;
    public static boolean orientHelp = false;
    private double previousX = 5, previousY = 5, previousR = 5;
    private boolean isRed = true;

    private enum IntakeControlsState {
        INTAKING, // claw on the floor and open for pixels
        WAITING_1,
        WAITING_2,
        CLOSED, // claw up and closed, holding pixels
        //        DELAY, // arm is moving, wait 0.25 seconds before moving claw subsystems
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

        waitingOneHalfSeconds = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(0.5)
                .build();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Press the START button");
        telemetry.update();

        waitForStart();

        robot.drivetrainSubsystem.resetInitYaw();

        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
        robot.armSubsystem.pivotState = Arm.PivotState.REST;
        robot.armSubsystem.armState = Arm.ArmState.NOTHING;

        if (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                drivetrainControls();

                intakeControls();

                telemetry.addLine("<D-Pad Up> \t Use the claw");
                telemetry.addLine("<Left/Right Bumpers> \t Open and close the claw");
                telemetry.addLine("<Joysticks> \t Move the robot");
//                telemetry.addData("intake state", intakeState);
//                telemetry.addData("drive is busy", drive.isBusy());
                telemetry.update();
                drive.update();
                robot.update();

            }
        }

    }

    public void drivetrainControls() {
        double x = gamepad1.left_stick_x * 1.05;
        double y = gamepad1.left_stick_y * -1;
        double r = gamepad1.right_stick_x;

        x /= 2.5;
        y /= 2.5;
        r /= 2.5;

        // attempting to save motor calls == faster frequency of command calls
        if ( !(previousX == x && previousY == y && previousR == r) ) {
            robot.drivetrainSubsystem.moveXYR(x, y, r, true);
        }

        previousX = x;
        previousY = y;
        previousR = r;
    }

    public void clawSidePieceControls(boolean reversed) {
        if (reversed) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                rightClosed = !rightClosed;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                leftClosed = !leftClosed;
            }
        }else {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                leftClosed = !leftClosed;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
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
                // reset button
                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                if (currentGamepad1.right_bumper && currentGamepad1.left_bumper) {
                    intakeState = IntakeControlsState.RESET;
                }

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up /*  && withinRange(BulkReading.pArmLeftMotor, -5, 5) */ ) {
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    intakeState = IntakeControlsState.INTAKING;
                    robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                }
                break;
            case INTAKING:
                clawSidePieceControls(true);

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    intakeState = IntakeControlsState.CLOSED;
                    robot.armSubsystem.pivotState = Arm.PivotState.REST;
                }
                break;
//            case WAITING_1:
//                drive.followTrajectorySequenceAsync(waitingOneHalfSeconds);
//                if (!drive.isBusy()) {
////                    telemetry.addData("test1", "");
//                    intakeState = IntakeControlsState.CLOSED;
//                }
//                drive.update();
//                break;
//            case WAITING_2:
//                drive.followTrajectorySequenceAsync(waitingOneHalfSeconds);
//                if (!drive.isBusy()) {
////                    telemetry.addData("test2", "");
//                    intakeState = IntakeControlsState.INTAKING;
//                }
//                drive.update();
//                break;
            case NOTHING:
                break;
        }

    }

}
