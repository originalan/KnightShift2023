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

@Autonomous(name = "BlueClose1 (with arm)", group = "AUTO")
public class BlueClose1 extends AutoBase {

    private TrajectorySequence detectionTraj,
            backboardTraj, parkingTraj, waitingTraj1,
            waitingTraj3, waitingTraj2;

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

        startingPose = new Pose2d(12, 63.125, Math.toRadians(270)); // the front of the robot is the control hub, not the delivery box so we rotate this by 180 degrees
        PoseStorage.startingAutoPose = new Pose2d(12, 63.125, Math.toRadians(270)); // to prevent shadowing

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);
        PoseStorage.AUTO_SHIFT_DEGREES = 180.0;

        drive.setPoseEstimate(startingPose);
        buildTrajectories();

        while (opModeInInit()) {
            detectedSide = propDetectionProcessor.getDetectedSide();
            telemetry.addLine("Blue, starting closer to backstage");
            telemetry.addLine("Puts purple pixel in place, drops yellow on backdrop, parks");
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
                        if (!drive.isBusy() &&
                                withinMargin(robot.deliveryArmMotor.getCurrentPosition(), RobotSettings.ARM_ENCODER_TOP, 4)) {
                            autoState = AutoState.RELEASE_PIXEL;
                            robot.intake.intakeState = Intake.IntakeState.REVERSE;
                            drive.followTrajectorySequenceAsync(waitingTraj3);
                        }
                        break;
                    case RELEASE_PIXEL:
                        // robot is at backdrop, arm is in place, intake is in place, wait 2 seconds as intake reverses to release pixel
                        if (!drive.isBusy()) {
                            robot.deliveryArm.armState = DeliveryArm.ArmState.BOTTOM; // bring arm back down
                            robot.intake.intakeState = Intake.IntakeState.OFF; // turn off intake
                            autoState = AutoState.PARKING;
                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        // robot is moving to parking destination
                        if (!drive.isBusy()) {
                            autoState = AutoState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;

                }

                drive.update();
                robot.deliveryArm.update();
                robot.intake.update();

                transferPose();
                telemetry.update();

            }
//            drive.followTrajectorySequence(detectionTraj);

        }

    }

    /**
     * Determines path based on location of prop
     */
    public void setGoalPose() {

        double distanceFromCenterToPurplePixel = 2;
        switch (detectedSide) {
            case LEFT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(23 - distanceFromCenterToPurplePixel, 35.25 + 17.75/2.0 - 2.5 ), Math.toRadians(270))
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineTo(new Vector2d(47 + 12.25 - 17.75/2.0 - 0.5,
                                35.25 + 6 + distanceFromCenterToPurplePixel), Math.toRadians(0))
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

        waitingTraj3 = drive.trajectorySequenceBuilder(backboardTraj.end())
                .waitSeconds(2)
                .build();

        parkingTraj = drive.trajectorySequenceBuilder(waitingTraj3.end())
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

    public boolean withinMargin(double first, double second, double margin) {

        if (Math.abs(first - second) <= margin) {
            return true;
        }
        return false;

    }

}
