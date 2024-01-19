package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static org.firstinspires.ftc.teamcode.util.RobotSettings.AUTO_PURPLE_PIXEL_RELEASE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@Autonomous(name = "Blue (Closer to backboard) Test", group = "Testing")
public class BlueTest extends AutoBase {

    private TrajectorySequence detectionTraj,
            backboardTraj, parkingTraj, waitingTraj1,
            forceIntoBackboardTraj, waitingTraj2;

    public enum AutoState {
        ORIENT_PURPLE_PIXEL,
        PLACING_PURPLE_PIXEL,
        LIFT_ARM_SLIGHTLY,
        MOVING_TO_BACKBOARD,
        RELEASE_PIXEL,
        PARKING,
        IDLE
    }

    public AutoState autoState = AutoState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(11.75, 61.5, Math.toRadians(270));
        PoseStorage.startingAutoPose = new Pose2d(11.75, 61.5, Math.toRadians(270)); // to prevent shadowing

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);

        drive.setPoseEstimate(startingPose);
        buildTrajectories();

        while (opModeInInit()) {
            detectedSide = propDetectionProcessor.getDetectedSide();
            robot.deliveryArm.armState = DeliveryArm.ArmState.BOTTOM;
            telemetry.addData("Detected", detectedSide);
            telemetry.update();
        }

        waitForStart();

        autoState = AutoState.ORIENT_PURPLE_PIXEL;
        drive.followTrajectorySequenceAsync(detectionTraj);

        if (opModeIsActive()) {

            while (opModeIsActive() && !isStopRequested()) {

                switch (autoState) {

                    case ORIENT_PURPLE_PIXEL:
                        // robot is moving to the purple pixel location
                        robot.intake.intakeState = Intake.IntakeState.OFF;
                        if (!drive.isBusy()) {
                            autoState = AutoState.PLACING_PURPLE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case PLACING_PURPLE_PIXEL:
                        // robot is at the purple pixel location, waiting 4 seconds to spit out purple pixel
                        robot.intake.intakeState = Intake.IntakeState.REVERSE;
                        if (!drive.isBusy()) {
                            autoState = AutoState.LIFT_ARM_SLIGHTLY;
                            drive.followTrajectorySequenceAsync(waitingTraj2);
                        }
                        break;
                    case LIFT_ARM_SLIGHTLY:
                        // robot is still at purple pixel, waiting 2 seconds to turn off intake and lift up arm out of the way
                        robot.intake.intakeState = Intake.IntakeState.HOLDING_PIXEL;
                        robot.deliveryArm.armState = DeliveryArm.ArmState.LIFT;
                        if (!drive.isBusy()) {
                            autoState = AutoState.MOVING_TO_BACKBOARD;
                            drive.followTrajectorySequenceAsync(backboardTraj);
                        }
                        break;
                    case MOVING_TO_BACKBOARD:
                        // robot is moving to the backboard, moving arm into position
                        robot.deliveryArm.armState = DeliveryArm.ArmState.TOP;
                        if (!drive.isBusy() && robot.deliveryArmMotor.getCurrentPosition() == RobotSettings.ARM_ENCODER_TOP) {
                            autoState = AutoState.RELEASE_PIXEL;
                            drive.followTrajectorySequenceAsync(forceIntoBackboardTraj);
                        }
                        break;
                    case RELEASE_PIXEL:
                        if (!drive.isBusy()) {
                            robot.deliveryArm.armState = DeliveryArm.ArmState.BOTTOM;
                            autoState = AutoState.PARKING;
                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        if (!drive.isBusy()) {
                            autoState = AutoState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;

                }

                drive.update();
                robot.deliveryArm.update();

                transferPose();
                telemetry.update();

            }
            drive.followTrajectorySequence(detectionTraj);

        }

    }

    /**
     * Determines path based on location of prop
     */
    public void setGoalPose() {

        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(22.0 + 1.5, 29.5), Math.toRadians(270))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
//                            robot.purplePixel.drop();
                        })
                        .back(29.25)
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineToLinearHeading(new Pose2d(50, (-35.25 - 6 + 2.375) * -1, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(11.75 + 1.5, 22.0), Math.toRadians(270))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
//                            robot.purplePixel.drop();
                        })
                        .back(36.25)
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineToLinearHeading(new Pose2d(50, (-35.25 + 2.375) * -1, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
            case RIGHT: // I physically don't know how to do it without running into the truss so we have middle case instead
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(11.75 + 1.5, 22.0), Math.toRadians(270))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
//                            robot.purplePixel.drop();
                        })
                        .back(36.25)
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineToLinearHeading(new Pose2d(50, (-35.25 + 6 + 2.375) * -1, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
        }

    }

    public void buildTrajectories() {

        setGoalPose();

        forceIntoBackboardTraj = drive.trajectorySequenceBuilder(backboardTraj.end())
                .forward(5) // need to change value later
                .build();

        parkingTraj = drive.trajectorySequenceBuilder(forceIntoBackboardTraj.end())
                .waitSeconds(2) // give time for servo to do its thing
                .forward(10)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(40, 58.75))
                .turn(Math.toRadians(-90))
                .forward(58.75-40)
                .build();


        waitingTraj1 = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(4)
                .build();

        waitingTraj2 = drive.trajectorySequenceBuilder(waitingTraj1.end())
                .waitSeconds(2)
                .build();

    }

}
