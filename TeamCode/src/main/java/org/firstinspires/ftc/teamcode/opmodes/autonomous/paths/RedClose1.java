package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.ArmSettings;

@Autonomous (name = "RedClose 2+0", group = "Testing")
public class RedClose1 extends AutoBase {
    private TrajectorySequence detectionTraj, backdropTraj,parkingTraj;
    private TrajectorySequence moveBackLittle;
    private TrajectorySequence waitingOneHalfSeconds, waitingThreeSeconds, waitingHalfSeconds;
    private TrajectorySequence waitingTime;
    private double waitTime = 0;
    private enum AutoState {
        WAITING_TIME,
        GO_TO_SPIKE_MARK,
        PLACING_PURPLE_PIXEL,
        PLACING_PURPLE_PIXEL2,
        PLACING_PURPLE_PIXEL3,
        MOVING_TO_BACKBOARD,
        LIFT_ARM,
        MOVE_FORWARD,
        RELEASE_PIXEL,
        ARM_BACK_DOWN,
        PARKING,
        IDLE
    }
    private AutoState state = AutoState.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(redCloseStart.getX(), redCloseStart.getY(), redCloseStart.getHeading());

        initialize(JVBoysSoccerRobot.AllianceType.RED);

        drive.setPoseEstimate(startingPose);
        detectedSide = propDetectionProcessor.getDetectedSide();
        buildTrajectories();

        while (opModeInInit()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            previousDetectedSide = propDetectionProcessor.copyDetection(detectedSide);
            detectedSide = propDetectionProcessor.getDetectedSide();
            if (previousDetectedSide != detectedSide) {
                buildTrajectories(); // gonna kill the battery, but we gotta do it cuz they move team prop x seconds after init
            }
            telemetry.addData("LOCATION: ", detectedSide);
            telemetry.addData("Waiting Time", waitTime);
            telemetry.addLine("Red, starting closer to backstage");
            telemetry.addLine("purple pixel in RIGHT claw!!!!!");
            telemetry.update();

            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                waitTime--;
            }
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                waitTime++;
            }
            if (waitTime < 0) {
                waitTime = 0;
            }
//            if (runtime.seconds() > 1.0 && checkAgain) {
//                checkAgain = false;
//                buildTrajectories();
//            }
        }

        waitForStart();

        waitingTime = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(waitTime)
                .build();

        state = AutoState.WAITING_TIME;
        drive.followTrajectorySequenceAsync(waitingTime);

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                switch (state) {
                    case WAITING_TIME:
                        // robot is waiting a certain amount of time
                        if (!drive.isBusy()) {
                            state = AutoState.GO_TO_SPIKE_MARK;
                            drive.followTrajectorySequenceAsync(detectionTraj);
                        }
                        break;
                    case GO_TO_SPIKE_MARK:
                        // robot is moving to the purple pixel location
                        robot.armSubsystem.pivotState = Arm.PivotState.REST;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL:
                        // robot is pivoting claw down for 0.5 seconds
                        robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL2;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL2:
                        // robot drops pixel for 0.5 secnods
                        robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL3;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL3:
                        // robot puts claw back up for 0.5 seconds
                        robot.armSubsystem.pivotState = Arm.PivotState.REST;
//                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                        if (!drive.isBusy()) {
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                            state = AutoState.MOVING_TO_BACKBOARD;
                            drive.followTrajectorySequenceAsync(backdropTraj);

                            robot.armSubsystem.encoderGoalPosition = ArmSettings.positionYellowPixel;
                            robot.armSubsystem.setMotionProfile();
                            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                        }
                        break;
                    case MOVING_TO_BACKBOARD:
                        // robot is moving to backboard, arm and servo are moving too
                        if (!drive.isBusy()) {
                            state = AutoState.LIFT_ARM;

                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                            drive.followTrajectorySequenceAsync(waitingThreeSeconds);
                        }
                        break;
                    case LIFT_ARM:
                        // give about 3.0 seconds for arm and servo to move in place
                        if (!drive.isBusy()) {
                            state = AutoState.MOVE_FORWARD;
                            drive.followTrajectorySequenceAsync(moveBackLittle);
                        }
                        break;
                    case MOVE_FORWARD:
                        // robot is moving back into the backdrop a little bit
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case RELEASE_PIXEL:
                        // 0.5 seconds for yellow pixel to release and fall
                        robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.ARM_BACK_DOWN;

                            robot.armSubsystem.encoderGoalPosition = ArmSettings.positionBottom;
                            robot.armSubsystem.setMotionProfile();
                            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;

                            robot.armSubsystem.pivotState = Arm.PivotState.REST;
                            drive.followTrajectorySequenceAsync(waitingThreeSeconds);
                        }
                        break;
                    case ARM_BACK_DOWN:
                        // robot is bringing arm back down for 3.0 seconds
                        if (!drive.isBusy()) {
                            state = AutoState.PARKING;
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        if (!drive.isBusy()) {
                            state = AutoState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;
                }

                drive.update();
                robot.armSubsystem.update();
                robot.clawSubsystem.update();

                robot.armSubsystem.addTelemetry();
                robot.clawSubsystem.addTelemetry();
                telemetry.update();

                transferPose();
            }
        }
    }

    public void buildTrajectories() {
        setGoalPose();

        waitingOneHalfSeconds = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(1.5)
                .build();
        waitingThreeSeconds = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(3.0)
                .build();
        waitingHalfSeconds = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(0.5)
                .build();
        moveBackLittle = drive.trajectorySequenceBuilder(backdropTraj.end())
                .back(5.5)
                .build();
        parkingTraj = drive.trajectorySequenceBuilder(moveBackLittle.end())
                .strafeTo(new Vector2d(backdropTraj.end().getX(), -46))
                .back(17)
                .build();
    }

    public void setGoalPose() {
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(startingPose.getX(), -36.0 - 1.725 + 0.5 + 8))
                        .turn(Math.toRadians(90))
//                        .splineTo(new Vector2d(2.25 + 0.5, -36.0 - 1.725 + 0.5), Math.toRadians(180))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, -49.5 + 20.25 + 1.725 + 8), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(15 - 1.725, -24.5 - 2.25), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, -49.5 + 20.25 + 1.725 - 6.0 + 8), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(23 - 1.725, -36.0 - 2.25 + 1), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, -49.5 + 20.25 + 1.725 - 12.0 + 8), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
        }
    }
}
