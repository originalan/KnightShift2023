package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;

@Autonomous (name = "BlueFar 2+0", group = "Testing")
public class BlueFar1 extends AutoBase {
    private TrajectorySequence detectionTraj, backdropTraj, parkingTraj;
    private TrajectorySequence moveBackLittle;
    private TrajectorySequence waitingOneAndHalfSeconds, waitingThreeSeconds, waitingHalfSecond;
    private TrajectorySequence waitingTime;
    private double waitTime = 0;
    private double shift = 9.5;
    private boolean leftSide = false;
    private boolean parkOutside = false;
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

        startingPose = new Pose2d(blueFarStart.getX(), blueFarStart.getY(), blueFarStart.getHeading());

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);

        drive.setPoseEstimate(startingPose);
        detectedSide = propDetectionProcessor.getDetectedSide();
        buildTrajectories();

        while (opModeInInit()) {
            previousDetectedSide = propDetectionProcessor.copyDetection(detectedSide);
            detectedSide = propDetectionProcessor.getDetectedSide();
            if (previousDetectedSide != detectedSide) {
                buildTrajectories(); // gonna kill the battery, but we gotta do it cuz they move team prop x seconds after init
            }
            telemetry.addData("LOCATION: ", detectedSide);
            telemetry.addLine("Blue, starting far to backstage");
            telemetry.addLine("purple pixel in LEFT claw!!!!!");
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
                    shift = 7.0;
                }else {
                    shift = 9.5;
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
                            drive.followTrajectorySequenceAsync(waitingHalfSecond);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL:
                        // robot is releasing claw servo for purple pixel
                        robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL2;
                            drive.followTrajectorySequenceAsync(waitingHalfSecond);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL2:
                        // robot drops pixel for 0.5 secnods
                        robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL3;
                            drive.followTrajectorySequenceAsync(waitingHalfSecond);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL3:
                        // robot puts claw back up for 0.5 seconds
                        robot.armSubsystem.pivotState = Arm.PivotState.REST;
                        if (!drive.isBusy()) {
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                            state = AutoState.MOVING_TO_BACKBOARD;
                            drive.followTrajectorySequenceAsync(backdropTraj);
                        }
                        break;
                    case MOVING_TO_BACKBOARD:
                        // robot is moving to backboard, arm and servo are moving too
                        if (!drive.isBusy()) {
                            state = AutoState.LIFT_ARM;

                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                            drive.followTrajectorySequenceAsync(waitingThreeSeconds);

                            robot.armSubsystem.setMotionProfile(ArmSettings.positionYellowPixel);
                            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                        }
                        break;
                    case LIFT_ARM:
                        // give about 3 seconds for arm and servo to move in place
                        if (!drive.isBusy()) {
                            state = AutoState.MOVE_FORWARD;
                            drive.followTrajectorySequenceAsync(moveBackLittle);
                        }
                        break;
                    case MOVE_FORWARD:
                        // robot is moving back into the backdrop a little bit
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingHalfSecond);
                        }
                        break;
                    case RELEASE_PIXEL:
                        // 0.5 seconds for yellow pixel to release and fall
                        robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.ARM_BACK_DOWN;

                            robot.armSubsystem.setMotionProfile(ArmSettings.positionBottom);
                            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;

                            robot.armSubsystem.pivotState = Arm.PivotState.REST;
                            drive.followTrajectorySequenceAsync(waitingThreeSeconds);
                        }
                        break;
                    case ARM_BACK_DOWN:
                        // robot is bringing arm back down for 3.0 seconds
                        if (!robot.armSubsystem.getMP().isBusy()) {
                            robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                        }
                        if (!drive.isBusy()) {
                            state = AutoState.PARKING;
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        if (!robot.armSubsystem.getMP().isBusy()) {
                            robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                        }
                        if (!drive.isBusy()) {
                            state = AutoState.IDLE;
                        }
                        break;
                    case IDLE:
                        if (!robot.armSubsystem.getMP().isBusy()) {
                            robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                        }
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

        waitingOneAndHalfSeconds = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(1.5)
                .build();
        waitingThreeSeconds = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(3.0)
                .build();
        waitingHalfSecond = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(0.5)
                .build();
        moveBackLittle = drive.trajectorySequenceBuilder(backdropTraj.end())
                .setReversed(false)
                .back(5)
                .build();
        buildParkTrajectory();
    }

    public void buildParkTrajectory() {
        if (parkOutside) {
            parkingTraj = drive.trajectorySequenceBuilder(moveBackLittle.end())
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(moveBackLittle.end().getX(), 54))
                    .turn(Math.toRadians(-90))
//                .strafeTo(new Vector2d(moveBackLittle.end().getX(), 46))
                    .back(15)
                    .build();
        }else {
            parkingTraj = drive.trajectorySequenceBuilder(moveBackLittle.end())
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(moveBackLittle.end().getX(), 3))
                    .turn(Math.toRadians(-90))
//                .strafeTo(new Vector2d(moveBackLittle.end().getX(), 46))
                    .back(15)
                    .build();
        }
    }

    public void setGoalPose() {
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(startingPose.getX(), (-36.0 - 1.725 + 0.5 + 12) * -1))
                        .turn(Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .turn(-1 * Math.toRadians(90))
                        .splineTo(new Vector2d(-36, (-3) * -1), Math.toRadians(180))
                        .forward(-48)
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, (-49.5 + 20.25 + 1.725 + shift) * -1), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(-39 - 1.725, (-24 - 0.5 - 2.25) * -1), Math.toRadians(270))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .back(1)
                        .turn(-1 * Math.toRadians(90))
                        .forward(-48)
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, (-49.5 + 20.25 + 1.725 - 6.0 + shift) * -1), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
//                        .splineTo(new Vector2d(-48 + 0.5 - 1.725, -36.0 - 2.25 + 1), Math.toRadians(90))
//                        .splineTo(new Vector2d(2.25 + 0.5, -36.0 - 1.725 + 0.5), Math.toRadians(180))
                        .lineTo(new Vector2d(startingPose.getX(), (-36.0 - 1.725 + 0.5 + 10) * -1))
                        .turn(-1 * Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
//                        .turn(Math.toRadians(90))
//                        .splineTo(new Vector2d(-36, -3), Math.toRadians(180))
//                        .forward(-48)
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(detectionTraj.end().getX(), (-3) * -1))
                        .turn(-1 * Math.toRadians(90))
                        .forward(-48)
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, (-49.5 + 20.25 + 1.725 - 12.0 + shift) * -1), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
        }
    }
}