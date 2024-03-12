package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Autonomous (name = "BlueClose 1+0", group = "Testing")
public class BlueClose2 extends AutoBase {
    private TrajectorySequence detectionTraj, backdropTraj, parkingTraj;
    private TrajectorySequence moveBackLittle;
    private TrajectorySequence waitingOneHalfSeconds, waitingThreeSeconds, waitingHalfSeconds;
    private TrajectorySequence waitingTime;
    private double waitTime = 0;
    private boolean park = true;
    private enum AutoState {
        WAITING_TIME,
        GO_TO_SPIKE_MARK,
        PLACING_PURPLE_PIXEL,
        PLACING_PURPLE_PIXEL2,
        PLACING_PURPLE_PIXEL3,
        PARKING,
        RELEASE_PIXEL,
        RELEASE_PIXEL2,
        RELEASE_PIXEL3,
        IDLE
    }
    private AutoState state = AutoState.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(blueCloseStart.getX(), blueCloseStart.getY(), blueCloseStart.getHeading());

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);

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
            telemetry.addLine("Blue, starting closer to backstage");
            telemetry.addLine("purple pixel in LEFT claw!!!!!");
            telemetry.addData("Parking? ", park);
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
                park = !park;
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
                        robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL3;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL3:
                        // robot puts claw back up for 0.5 seconds
                        robot.armSubsystem.pivotState = Arm.PivotState.REST;
                        if (!drive.isBusy()) {
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                            if (park) {
                                drive.followTrajectorySequenceAsync(backdropTraj);
                                state = AutoState.PARKING;
                            }else {
                                state = AutoState.IDLE;
                            }
                        }
                        break;
                    case PARKING:
                        // robot is moving to park
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL;
                            robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case RELEASE_PIXEL:
                        // 0.5 seconds for claw to pivot down
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL2;
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case RELEASE_PIXEL2:
                        // 0.5 seconds to release pixel
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL3;
                            robot.armSubsystem.pivotState = Arm.PivotState.REST;
                            drive.followTrajectorySequenceAsync(waitingHalfSeconds);
                        }
                        break;
                    case RELEASE_PIXEL3:
                        // 0.5 seconds to bring claw up
                        if (!drive.isBusy()) {
                            state = AutoState.IDLE;
                            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
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
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(moveBackLittle.end().getX(), 54))
                .turn(Math.toRadians(-90))
//                .strafeTo(new Vector2d(moveBackLittle.end().getX(), 46))
                .back(15)
                .build();
    }

    public void setGoalPose() {
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(23 - 1.725, (-36.0 - 2.25 + 1) * -1), Math.toRadians(270))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .turn(Math.toRadians(-90))
                        .setReversed(true)
                        .splineTo(new Vector2d(50, 60), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(15 - 1.725, (-24.5 - 2.25) * -1), Math.toRadians(270))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(50, 60), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(startingPose.getX(), (-36.0 - 1.725 + 0.5 + 8) * -1))
                        .turn(-1 * Math.toRadians(90))
//                        .splineTo(new Vector2d(2.25 + 0.5, -36.0 - 1.725 + 0.5), Math.toRadians(180))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(50, 60), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
        }
    }
}
