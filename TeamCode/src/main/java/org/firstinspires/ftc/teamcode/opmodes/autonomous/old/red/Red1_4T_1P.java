package org.firstinspires.ftc.teamcode.opmodes.autonomous.old.red;

import static org.firstinspires.ftc.teamcode.util.RobotSettings.AUTO_PURPLE_PIXEL_RELEASE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@Autonomous(name = "Red1_4T_1P", group = "Autonomous Opmode 11.19")
public class Red1_4T_1P extends AutoBase {

    private TrajectorySequence detectionTraj;
    private TrajectorySequence backboardTraj;
    private TrajectorySequence parkingTraj;

    public enum AutoState {
        PLACING_PURPLE_PIXEL,
        MOVING_TO_BACKBOARD,
        MANEUVER_ARM,
        OPEN_SERVO_CLAW,
        BRING_ARM_BACK,
        PARKING,
        IDLE
    }
    public AutoState autoState = AutoState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(11.75, -61.5, Math.toRadians(90));
        initialize(JVBoysSoccerRobot.AllianceType.RED);

        drive.setPoseEstimate(startingPose);
        buildTrajectories();

        while (opModeInInit()) {
            detectedSide = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectedSide);
            telemetry.update();
            sleep(1);
        }

        waitForStart();

        autoState = AutoState.IDLE;
        drive.followTrajectorySequence(detectionTraj);

        if (opModeIsActive()) {

            while (opModeIsActive() && !isStopRequested()) {

                switch (autoState) {

//                    case PLACING_PURPLE_PIXEL:
//                        robot.deliveryArm.slideState = DeliveryArm.ArmState.HOLDING_PIXEL;
//                        if (!drive.isBusy()) {
//                            autoState = AutoState.MOVING_TO_BACKBOARD;
//                            drive.followTrajectorySequenceAsync(backboardTraj);
//                        }
//                        break;
//                    case MOVING_TO_BACKBOARD:
//                        robot.deliveryArm.slideState = DeliveryArm.ArmState.HOLDING_PIXEL; // technically don't need this but whatever
//                        if (!drive.isBusy()) {
//                            autoState = AutoState.MANEUVER_ARM;
//                            robot.deliveryArmMotor.setTargetPosition(RobotSettings.OUTTAKE_MOTOR_ENCODER_POSITION);
//                            robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            robot.deliveryArm.slideState = DeliveryArm.ArmState.BRING_ARM_IN_PLACE;
//                        }
//                        break;
//                    case MANEUVER_ARM:
//                        if (!robot.deliveryArmMotor.isBusy()) {
//                            autoState = AutoState.OPEN_SERVO_CLAW;
//                            robot.deliveryArm.slideState = DeliveryArm.ArmState.RELEASE_PIXEL;
//                        }
//                        break;
//                    case OPEN_SERVO_CLAW:
//                        if (robot.linearSlideServo.getPosition() == RobotSettings.OUTTAKE_SERVO_CLAW_RELEASE_POSITION) {
//                            autoState = AutoState.BRING_ARM_BACK;
//                            robot.deliveryArmMotor.setTargetPosition(0);
//                            robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            robot.deliveryArm.slideState = DeliveryArm.ArmState.BRING_ARM_BACK;
//                        }
//                        break;
//                    case BRING_ARM_BACK:
//                        if (!robot.deliveryArmMotor.isBusy()) {
//                            autoState = AutoState.PARKING;
//                            robot.deliveryArm.slideState = DeliveryArm.ArmState.OFF;
//                            drive.followTrajectorySequenceAsync(parkingTraj);
//                        }
//                        break;
//                    case PARKING:
//                        if (!drive.isBusy()) {
//                            autoState = AutoState.IDLE;
//                            robot.deliveryArm.slideState = DeliveryArm.ArmState.OFF;
//                        }
//                        break;
//                    case IDLE:
//
//                        break;
                }

                drive.update();
                robot.deliveryArm.update();

                transferPose();
                telemetry.update();

            }

        }

        transferPose();

    }

    /**
     * Determines path based on location of prop
     */
    public void setGoalPose() {

        switch (detectedSide) {
            case LEFT: // I physically don't know how to do it without running into the truss so we have middle case instead
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(11.75 - 1.5, -22.0), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(36.25)
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineToLinearHeading(new Pose2d(50, -35.25 + 6 - 2.375, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(11.75 - 1.5, -22.0), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(36.25)
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineToLinearHeading(new Pose2d(50, -35.25 - 2.375, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(22.0 - 1.5, -29.5), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(29.25)
                        .build();
                backboardTraj = drive.trajectorySequenceBuilder(detectionTraj.end())
                        .splineToLinearHeading(new Pose2d(50, -35.25 - 6 - 2.375, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
        }

    }

    public void buildTrajectories() {

        setGoalPose();

        parkingTraj = drive.trajectorySequenceBuilder(backboardTraj.end())
                .waitSeconds(2)
                .forward(10)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(40, -58.75))
                .turn(Math.toRadians(90))
                .forward(58.75-40)
                .build();

    }

}
