package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous (name = "RedFar 2+1", group = "Testing")
public class RedFar1 extends AutoBase {
    private TrajectorySequence detectionTraj, orientPixelStackTraj, reorientTraj, backdropTraj, parkingTraj;
    private TrajectorySequence waitingTraj1, waitingTraj2;
    private boolean orientationReached = false;
    private enum AutoState {
        GO_TO_SPIKE_MARK,
        PLACING_PURPLE_PIXEL,
        ORIENT_PIXEL_STACK,
        CALCULATE_REORIENTATION,
        REORIENT_PIXEL_STACK,
        GO_TO_PIXEL_STACK,
        PICK_UP_PIXEL,
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
            telemetry.addLine("Puts purple pixel in place, gets 1 white pixel, drops both on backdrop, parks");
            telemetry.addLine("PURPLE PIXEL IN RIGHT CLAW!!!!!");
            telemetry.update();
        }

        waitForStart();

        portal.setProcessorEnabled(propDetectionProcessor, false);
        portal.setProcessorEnabled(aprilTagProcessor, true);

        state = AutoState.GO_TO_SPIKE_MARK;
        drive.followTrajectorySequenceAsync(detectionTraj);

        double x = orientPixelStackTraj.end().getX();
        double y = orientPixelStackTraj.end().getY();
        double heading = orientPixelStackTraj.end().getHeading();
        double yaw = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (!orientationReached) {
                    if (aprilTagProcessor.getDetections().size() > 0) {
                        for (AprilTagDetection tag : aprilTagProcessor.getDetections()) {
                            if (tag.id == 8) {
                                double angle1 = tag.ftcPose.bearing - tag.ftcPose.yaw;
                                heading = Math.toRadians(180 - tag.ftcPose.yaw);
                                yaw = tag.ftcPose.yaw;
                                x = -72.0 + (tag.ftcPose.range * Math.cos(Math.toRadians(angle1))) + (6.27 * Math.sin( Math.toRadians(tag.ftcPose.yaw) ));
                                y = -36.0 + (tag.ftcPose.range * Math.sin(Math.toRadians(angle1))) + (6.27 * Math.cos( Math.toRadians(tag.ftcPose.yaw) ));
//                                reorientTraj = drive.trajectorySequenceBuilder(new Pose2d(x, y, heading))
//                                        .splineTo(new Vector2d(-60.0, -36.0 - 1.725), Math.toRadians(180))
//                                        .forward(7)
//                                        .build();
//                                setBackdropTraj();
                            }
                        }
                    }
                }

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
                        robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.ORIENT_PIXEL_STACK;
                            drive.followTrajectorySequenceAsync(orientPixelStackTraj);
                        }
                        break;
                    case ORIENT_PIXEL_STACK:
                        // robot is setting up to intake one white pixel from pixel stack
                        robot.armSubsystem.armState = Arm.ArmState.AUTO_PIXEL_STACK_POS_2;
                        if (!drive.isBusy()) {
                            state = AutoState.CALCULATE_REORIENTATION;
                        }
                        break;
                    case CALCULATE_REORIENTATION:
                        // redo the next trajectory once
                        orientationReached = true;
                        reorientTraj = drive.trajectorySequenceBuilder(new Pose2d(x, y, heading))
                                .turn(yaw)
                                .lineTo(new Vector2d(-60.0, -36.0 - 1.725))
                                .forward(7)
                                .build();
                        setBackdropTraj();
                        state = AutoState.REORIENT_PIXEL_STACK;
                        drive.followTrajectorySequenceAsync(reorientTraj);
                        break;
                    case REORIENT_PIXEL_STACK:
                        // robot is reorienting using the apriltag for the pixel stack
                        if (!drive.isBusy()) {
                            state = AutoState.PICK_UP_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case PICK_UP_PIXEL:
                        // waiting 1.5 seconds to pick up from pixel stack
                        if (!drive.isBusy()) {
                            state = AutoState.MOVING_TO_BACKBOARD;
                            drive.followTrajectorySequenceAsync(backdropTraj);
                        }
                        break;
                    case MOVING_TO_BACKBOARD:
                        // robot is moving to the backdrop
                        if (!drive.isBusy()) {
                            state = AutoState.LIFT_ARM;
                            drive.followTrajectorySequenceAsync(waitingTraj2);
                        }
                        break;
                    case LIFT_ARM:
                        // give robot 3 seconds to lift arm into place
                        if (!drive.isBusy()) {
                            state = AutoState.RELEASE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case RELEASE_PIXEL:
                        // give robot 1.5 seconds to release pixels onto backdrop
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.PARKING;
                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                        robot.armSubsystem.armState = Arm.ArmState.BOTTOM_CLAW_UP;
                        if (!drive.isBusy()) {
                            state = AutoState.IDLE;
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

        reorientTraj = drive.trajectorySequenceBuilder(orientPixelStackTraj.end())
                .splineTo(new Vector2d(-60.0, -36.0 - 1.725), Math.toRadians(180))
                .forward(7)
                .build();

        waitingTraj1 = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(1.5)
                .build();
        waitingTraj2 = drive.trajectorySequenceBuilder(backdropTraj.end())
                .waitSeconds(3.0)
                .build();

        setBackdropTraj();

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
                orientPixelStackTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .turn(Math.toRadians(90))
                        .splineTo(new Vector2d(-60.0, -36.0 - 1.725), Math.toRadians(180))
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(-39 - 1.725, -24 - 0.5 - 2.25), Math.toRadians(90))
                        .build();
                orientPixelStackTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .turn(Math.toRadians(90))
                        .splineTo(new Vector2d(-60.0, -36.0 - 1.725), Math.toRadians(180))
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(startingPose.getX(), -48))
                        .splineTo(new Vector2d(-24.5 - 2.25, -36.0 + 1.725), Math.toRadians(0))
                        .build();
                orientPixelStackTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .turn(Math.toRadians(180))
                        .splineTo(new Vector2d(-60.0, -36.0 - 1.725), Math.toRadians(180))
                        .build();
                break;
        }
    }

    public void setBackdropTraj() {
        switch (detectedSide) {
            case LEFT:
                backdropTraj = drive.trajectorySequenceBuilder(reorientTraj.end())
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
                backdropTraj = drive.trajectorySequenceBuilder(reorientTraj.end())
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
                backdropTraj = drive.trajectorySequenceBuilder(reorientTraj.end())
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
