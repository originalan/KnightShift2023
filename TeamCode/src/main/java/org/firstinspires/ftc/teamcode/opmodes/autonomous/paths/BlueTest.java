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
import org.firstinspires.ftc.teamcode.util.FullStateFeedback;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@Autonomous(name = "Blue (Closer) Test", group = "Testing")
public class BlueTest extends AutoBase {

    private TrajectorySequence detectionTraj, backboardTraj, parkingTraj, waitingTraj, forceIntoBackboardTraj;
    private FullStateFeedback controller;

    public enum AutoState {
        ORIENT_PURPLE_PIXEL,
        PLACING_PURPLE_PIXEL,
        MOVING_TO_BACKBOARD,
        MANEUVER_ARM,
        RELEASE_PIXEL,
        BRING_ARM_BACK,
        PARKING,
        IDLE
    }

    public AutoState autoState = AutoState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new FullStateFeedback();

        startingPose = new Pose2d(11.75, 61.5, Math.toRadians(270));
        PoseStorage.startingAutoPose = new Pose2d(11.75, 61.5, Math.toRadians(270));

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);

        drive.setPoseEstimate(startingPose);
        buildTrajectories();

        while (opModeInInit()) {
            detectedSide = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectedSide);
            telemetry.update();
        }

        waitForStart();

        autoState = AutoState.ORIENT_PURPLE_PIXEL;
        drive.followTrajectorySequenceAsync(detectionTraj);

        if (opModeIsActive()) {

            while (opModeIsActive() && !isStopRequested()) {

//                switch (autoState) {
//
//                    case ORIENT_PURPLE_PIXEL:
//                        robot.intake.intakeState = Intake.IntakeState.OFF;
//                        if (!drive.isBusy()) {
//                            autoState = AutoState.PLACING_PURPLE_PIXEL;
//                            drive.followTrajectorySequenceAsync(waitingTraj);
//                        }
//                        break;
//                    case PLACING_PURPLE_PIXEL:
//                        if (!drive.isBusy()) {
//                            autoState = AutoState.MOVING_TO_BACKBOARD;
//                            drive.followTrajectorySequenceAsync(backboardTraj);
//                        }
//                        break;
//                    case MOVING_TO_BACKBOARD:
//                        robot.deliveryArm.slideState = DeliveryArm.ArmState.TOP;
//                        if (!drive.isBusy() && robot.deliveryArmMotor.getCurrentPosition() == RobotSettings.OUTTAKE_MOTOR_ENCODER_POSITION) {
//                            autoState = AutoState.RELEASE_PIXEL;
//                            drive.followTrajectorySequenceAsync(forceIntoBackboardTraj);
//                        }
//                        break;
//                    case MANEUVER_ARM:
////                        if (robot.deliveryArmMotor.getCurrentPosition() == RobotSettings.OUTTAKE_MOTOR_ENCODER_POSITION) {
////                            autoState = AutoState.RELEASE_PIXEL;
////                        }
//                        break;
//                    case RELEASE_PIXEL:
//                        if (!drive.isBusy()) {
//                            robot.deliveryArm.slideState = DeliveryArm.ArmState.BOTTOM;
//                            autoState = AutoState.PARKING;
//                            drive.followTrajectorySequenceAsync(parkingTraj);
//                        }
//                        break;
//                    case BRING_ARM_BACK:
////                        if (!robot.deliveryArmMotor.isBusy()) {
////                            autoState = AutoState.PARKING;
////                            robot.deliveryArm.slideState = DeliveryArm.ArmState.OFF;
////                            drive.followTrajectorySequenceAsync(parkingTraj);
////                        }
//                        break;
//                    case PARKING:
//                        if (!drive.isBusy()) {
//                            autoState = AutoState.IDLE;
//                        }
//                        break;
//                    case IDLE:
//                        break;
//
//                }

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


        waitingTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                .waitSeconds(4)
                .build();

    }

}
