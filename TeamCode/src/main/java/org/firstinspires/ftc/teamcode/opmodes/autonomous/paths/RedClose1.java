package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoSettings;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoSettings.*;

@Autonomous (name = "RedClose 2+0", group = "Testing")
public class RedClose1 extends AutoBase {
    private TrajectorySequence detectionTraj, backdropTraj, parkingTraj;
    private TrajectorySequence moveBackLittle;
    private TrajectorySequence waitingOneHalfSeconds, waitingThreeSeconds, waitingHalfSeconds, waitingQuarterSeconds;
    private TrajectorySequence waitingTime;
    private double waitTime = 0;
    private double shift = AutoSettings.leftYellow;
    private boolean leftSide = true;
    private boolean parkOutside = true;
    private boolean armMoving = false;
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
            telemetry.addLine("purple pixel in RIGHT claw");
            telemetry.addData("Yellow Pixel on", leftSide ? "LEFT" : "RIGHT");
            telemetry.addData("Parking on ", parkOutside ? "OUTSIDE" : "INSIDE");
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
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                leftSide = !leftSide;
                if (leftSide) {
                    shift = AutoSettings.leftYellow;
                } else {
                    shift = AutoSettings.rightYellow;
                }
                buildTrajectories();
            }
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                parkOutside = !parkOutside;
                buildParkTrajectory();
            }
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

                if (!armMoving) {
                    stateMachine();
                    drive.update();
                }else {
                    if (!drive.isBusy() && !robot.armSubsystem.getMP().isBusy()) {
                        armMoving = false;
                    }
                }

                robot.armSubsystem.update();
                robot.clawSubsystem.update();

                robot.armSubsystem.addTelemetry();
                robot.clawSubsystem.addTelemetry();
                telemetry.update();

                transferPose();
            }
        }
    }

    public void stateMachine() {
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
                    drive.followTrajectorySequenceAsync(waitingQuarterSeconds);
                }
                break;
            case PLACING_PURPLE_PIXEL:
                // robot is pivoting claw down for 0.5 seconds
                robot.armSubsystem.pivotState = Arm.PivotState.PURPLE;
                if (!drive.isBusy()) {
                    state = AutoState.PLACING_PURPLE_PIXEL2;
                    drive.followTrajectorySequenceAsync(waitingQuarterSeconds);
                }
                break;
            case PLACING_PURPLE_PIXEL2:
                // robot drops pixel for 0.5 secnods
                robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
                if (!drive.isBusy()) {
                    state = AutoState.PLACING_PURPLE_PIXEL3;
                    drive.followTrajectorySequenceAsync(waitingQuarterSeconds);
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

//                    robot.armSubsystem.setMotionProfile(ArmSettings.positionYellowPixel);
//                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
//                    armMoving = true;
                }
                break;
            case MOVING_TO_BACKBOARD:
                // robot is moving to backboard, arm and servo are moving too
                if (!drive.isBusy()) {
                    state = AutoState.LIFT_ARM;

                    robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                    robot.armSubsystem.setMotionProfile(ArmSettings.positionYellowPixel);
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    armMoving = true;

//                    drive.followTrajectorySequenceAsync(waitingThreeSeconds);
                }
                break;
            case LIFT_ARM:
                // give about 3.0 seconds for arm and servo to move in place
//                if (!drive.isBusy()) {
//                    state = AutoState.MOVE_FORWARD;
//                    drive.followTrajectorySequenceAsync(moveBackLittle);
//                }
                if (!robot.armSubsystem.getMP().isBusy()) {
                    state = AutoState.MOVE_FORWARD;
                    drive.followTrajectorySequenceAsync(moveBackLittle);
                }
                break;
            case MOVE_FORWARD:
                // robot is moving back into the backdrop a little bit
                if (!drive.isBusy()) {
                    state = AutoState.RELEASE_PIXEL;
                    drive.followTrajectorySequenceAsync(waitingQuarterSeconds);
                }
                break;
            case RELEASE_PIXEL:
                // 0.5 seconds for yellow pixel to release and fall
                robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
                if (!drive.isBusy()) {
                    state = AutoState.ARM_BACK_DOWN;

                    robot.armSubsystem.setMotionProfile(ArmSettings.positionBottom);
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;

                    robot.armSubsystem.pivotState = Arm.PivotState.REST;
                    armMoving = true;
                }
                break;
            case ARM_BACK_DOWN:
                // robot is bringing arm back down
                if (!robot.armSubsystem.getMP().isBusy()) {
                    robot.armSubsystem.setArmPower(0);
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                    state = AutoState.PARKING;

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
        waitingQuarterSeconds = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(0.25)
                .build();
        moveBackLittle = drive.trajectorySequenceBuilder(backdropTraj.end())
                .back(AutoSettings.moveBack)
                .build();
        buildParkTrajectory();
    }

    public void buildParkTrajectory() {
        if (parkOutside) {
            parkingTraj = drive.trajectorySequenceBuilder(moveBackLittle.end())
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(moveBackLittle.end().getX(), AutoSettings.parkingOuterY))
                    .turn(Math.toRadians(90))
//                .strafeTo(new Vector2d(moveBackLittle.end().getX(), 46))
                    .back(15)
                    .build();
        }else {
            parkingTraj = drive.trajectorySequenceBuilder(moveBackLittle.end())
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(moveBackLittle.end().getX(), AutoSettings.parkingInnerY))
                    .turn(Math.toRadians(90))
//                .strafeTo(new Vector2d(moveBackLittle.end().getX(), 46))
                    .back(15)
                    .build();
        }
    }

    public void setGoalPose() {
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(AutoSettings.leftDetectionX, AutoSettings.leftDetectionY), startingPose.getHeading())
                        .turn(Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(AutoSettings.leftBackdropX, AutoSettings.leftBackdropY + shift), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(AutoSettings.middleDetectionX, AutoSettings.middleDetectionY), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(AutoSettings.middleBackdropX, AutoSettings.middleBackdropY + shift), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(AutoSettings.rightDetectionX, AutoSettings.rightDetectionY), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(AutoSettings.rightBackdropX, AutoSettings.rightBackdropY + shift), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
        }
    }
}
