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

@Autonomous (name = "RedClose 2+2", group = "Testing")
public class RedClose2 extends AutoBase {

    private TrajectorySequence detectionTraj, backdropTraj, parkingTraj;
    private TrajectorySequence goToStack, goBackFromStack;
    private TrajectorySequence waitingTraj1, waitingTraj2;
    private enum AutoState {

        GO_TO_SPIKE_MARK,
        PLACE_PURPLE_PIXEL,
        MOVE_TO_BACKBOARD1,
        LIFT_ARM,
        PLACE_YELLOW_PIXEL,
        GO_TO_STACK,
        INTAKE_STACK,
        MOVE_TO_BACKBOARD2,
        RELEASE_WHITE_PIXELS,
        PARKING,
        IDLE
    }

    private AutoState state = AutoState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        // 17.7 x 17.25 inches
        double x = 12;
        double y = -54.3;
        double heading = Math.toRadians(90);
        startingPose = new Pose2d(x, y, heading);
        PoseStorage.startingAutoPose = new Pose2d(x, y, heading); // to prevent shadowing

        initialize(JVBoysSoccerRobot.AllianceType.RED);
        PoseStorage.AUTO_SHIFT_DEGREES = 0;

        drive.setPoseEstimate(startingPose);
        detectedSide = propDetectionProcessor.getDetectedSide();
        buildTrajectories();

        while (opModeInInit()) {
            detectedSide = propDetectionProcessor.getDetectedSide();
            telemetry.addLine("Red, starting closer to backstage");
            telemetry.addLine("Puts purple pixel in place, drops yellow on backdrop, parks");
            telemetry.addLine("PURPLE PIXEL IN RIGHT CLAW!!!!!");
            telemetry.update();
        }

        waitForStart();

        state = AutoState.GO_TO_SPIKE_MARK;
        drive.followTrajectorySequenceAsync(detectionTraj);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                switch (state) {
                    case GO_TO_SPIKE_MARK:
                        // robot is moving to the purple pixel location
                        robot.armSubsystem.armState = Arm.ArmState.BOTTOM_CLAW_UP;
                        if (!drive.isBusy()) {
                            state = AutoState.PLACE_PURPLE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case PLACE_PURPLE_PIXEL:
                        // robot is releasing claw servo for purple pixel
                        robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.MOVE_TO_BACKBOARD1;
                            drive.followTrajectorySequenceAsync(backdropTraj);
                        }
                        break;
                    case MOVE_TO_BACKBOARD1:
                        // robot is moving to backboard, arm and servo are moving too
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                        robot.armSubsystem.armState = Arm.ArmState.AUTO_YELLOW_POS;
                        if (!drive.isBusy()) {
                            state = AutoState.LIFT_ARM;
                            drive.followTrajectorySequenceAsync(waitingTraj2);
                        }
                        break;
                    case LIFT_ARM:
                        // give about 3 seconds for arm and servo to move in place
                        if (!drive.isBusy()) {
                            state = AutoState.PLACE_YELLOW_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case PLACE_YELLOW_PIXEL:
                        // 1.0 seconds for yellow pixel to release and fall
                        robot.clawSubsystem.clawState = Claw.ClawState.LEFT_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.GO_TO_STACK;
                            drive.followTrajectorySequenceAsync(goToStack);
                        }
                        break;
                    case GO_TO_STACK:
                        robot.armSubsystem.armState = Arm.ArmState.AUTO_PIXEL_STACK_POS;
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.INTAKE_STACK;
                            drive.followTrajectorySequenceAsync(waitingTraj2);
                        }
                        break;
                    case INTAKE_STACK:
                        // wait 3 seconds to get the pixels
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                        if (!drive.isBusy()) {
                            state = AutoState.MOVE_TO_BACKBOARD2;
                            drive.followTrajectorySequenceAsync(goBackFromStack);
                        }
                        break;
                    case MOVE_TO_BACKBOARD2:
                        // going back, lifts arm after crossing the truss
                        if (!drive.isBusy()) {
                            state = AutoState.MOVE_TO_BACKBOARD2;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case RELEASE_WHITE_PIXELS:
                        // 1.0 seconds for white pixel to release and fall
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
                        if (!drive.isBusy()) {
                            state = AutoState.PARKING;
                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        robot.armSubsystem.armState = Arm.ArmState.BOTTOM_CLAW_UP;
                        robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
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
                .waitSeconds(1.0)
                .build();
        waitingTraj2 = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(3.0)
                .build();
        goToStack = drive.trajectorySequenceBuilder(backdropTraj.end())
                .splineTo(new Vector2d(12, -12 + 1.725), Math.toRadians(180))
                .forward(79.5)
                .build();
        goBackFromStack = drive.trajectorySequenceBuilder(goToStack.end())
                .UNSTABLE_addDisplacementMarkerOffset(72, () -> {
                    robot.armSubsystem.armState = Arm.ArmState.AUTO_YELLOW_POS;
                })
                .back(79.5)
                .setReversed(true)
                .splineTo(new Vector2d(60.75 - 34.25 - 0.25, -49.5 + 20.25 + 1.725 - 6.0), Math.toRadians(0))
                .setReversed(false)
                .build();
        parkingTraj = drive.trajectorySequenceBuilder(goBackFromStack.end())
                .strafeTo(new Vector2d(goBackFromStack.end().getX(), -60))
                .back(40)
                .build();
    }

    public void setGoalPose() {
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(2.25 + 0.5, -36.0 - 1.725 + 0.5), Math.toRadians(180))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 34.25 - 0.25, -49.5 + 20.25 + 1.725), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(15 - 1.725, -24.5 - 2.25), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 34.25 - 0.25, -49.5 + 20.25 + 1.725 - 6.0), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(23 - 1.725, -36.0 - 2.25 + 1), Math.toRadians(90))
                        .build();
                backdropTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(60.75 - 34.25 - 0.25, -49.5 + 20.25 + 1.725 - 12.0), Math.toRadians(0))
                        .setReversed(false)
                        .build();
                break;
        }
    }
}
