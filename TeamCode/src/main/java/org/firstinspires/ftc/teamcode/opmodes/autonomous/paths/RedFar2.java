package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous (name = "RedFar 2+0", group = "Testing")
public class RedFar2 extends AutoBase {
    private TrajectorySequence detectionTraj, backdropTraj, parkingTraj;
    private TrajectorySequence waitingTraj1, waitingTraj2;
    private enum AutoState {
        GO_TO_SPIKE_MARK,
        PLACING_PURPLE_PIXEL,
        MOVING_TO_BACKBOARD,
        LIFT_ARM,
        RELEASE_PIXEL,
        PARKING,
        IDLE
    }
    private AutoState state = AutoState.IDLE;
    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(redFarStart.getX(), redFarStart.getY(), redFarStart.getHeading());

        initialize(JVBoysSoccerRobot.AllianceType.RED);

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
            telemetry.addLine("Red, starting far to backstage");
            telemetry.addLine("Puts purple pixel in place, drops yellow on backdrop, parks");
            telemetry.addLine("PURPLE PIXEL IN RIGHT CLAW!!!!!");
            telemetry.update();
//            if (runtime.seconds() > 1.0 && checkAgain) {
//                checkAgain = false;
//                buildTrajectories();
//            }
        }

        waitForStart();

        state = AutoState.GO_TO_SPIKE_MARK;
        drive.followTrajectorySequenceAsync(detectionTraj);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                switch (state) {
                    case GO_TO_SPIKE_MARK:
                        // robot is moving to the purple pixel location
                        robot.armSubsystem.armState = Arm.ArmState.BOTTOM_CLAW_DOWN;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACING_PURPLE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL:
                        // robot is releasing claw servo for purple pixel
                        robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.MOVING_TO_BACKBOARD;
                            drive.followTrajectorySequenceAsync(backdropTraj);
                        }
                        break;
                    case MOVING_TO_BACKBOARD:
                        // robot is moving to backboard, arm and servo are moving too
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
//                        robot.armSubsystem.armState = Arm.ArmState.AUTO_YELLOW_POS;
//                        done in trajectory marker
                        if (!drive.isBusy()) {
                            state = AutoState.LIFT_ARM;
                            drive.followTrajectorySequenceAsync(waitingTraj2);
                        }
                        break;
                    case LIFT_ARM:
                        // give about 3 seconds for arm and servo to move in place
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case RELEASE_PIXEL:
                        // 1.5 seconds for yellow pixel to release and fall
                        robot.clawSubsystem.clawState = Claw.ClawState.LEFT_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.PARKING;
                            robot.armSubsystem.armState = Arm.ArmState.BOTTOM_CLAW_UP;
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

        waitingTraj1 = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(1.5)
                .build();
        waitingTraj2 = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(3.0)
                .build();
        parkingTraj = drive.trajectorySequenceBuilder(backdropTraj.end())
                .strafeTo(new Vector2d(backdropTraj.end().getX(), -12))
                .back(40)
                .build();
    }

    public void setGoalPose() {
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(-48 + 0.5 - 1.725, -36.0 - 2.25), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                        .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                            robot.armSubsystem.armState = Arm.ArmState.AUTO_YELLOW_POS;
                        })
                        .forward(48)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25, -49.5 + 20.25 + 1.725), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(-39 - 1.725, -24 - 0.5 - 2.25), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                        .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                            robot.armSubsystem.armState = Arm.ArmState.AUTO_YELLOW_POS;
                        })
                        .forward(48)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25, -49.5 + 20.25 + 1.725 - 6.0), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(startingPose.getX(), -48))
                        .splineTo(new Vector2d(-24.5 - 2.25, -36.0 + 1.725), Math.toRadians(0))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                        .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                            robot.armSubsystem.armState = Arm.ArmState.AUTO_YELLOW_POS;
                        })
                        .forward(48)
                        .splineTo(new Vector2d(60.75 - 32.5 - 0.25, -49.5 + 20.25 + 1.725 - 12.0), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
        }
    }
}
