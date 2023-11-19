package org.firstinspires.ftc.teamcode.opmodes.autonomous.blue;

import static org.firstinspires.ftc.teamcode.util.RobotSettings.AUTO_PURPLE_PIXEL_RELEASE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@TeleOp(name = "Blue2_2T_1P (places pixel, parks inner)", group = "Autonomous Opmode 11.19")
public class Blue2_2T_1P extends AutoBase {

    private TrajectorySequence traj1;
    private TrajectorySequence detectionTraj;

    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(-35.25, 61.5, Math.toRadians(270));
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

        if (opModeIsActive()) {

            drive.followTrajectorySequence(detectionTraj);

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
                        .splineTo(new Vector2d(-35.25 - 1.5, 25.0), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(13.25)
                        .turn(Math.toRadians(-90))
                        .forward(34.75 + 58.75)
                        .build();
                break;
            case MIDDLE:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(-35.25 - 1.5, 25.0), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(13.25)
                        .turn(Math.toRadians(-90))
                        .forward(34.75 + 58.75)
                        .build();
                break;
            case RIGHT:
                detectionTraj = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(-45.5 - 1.5, 29.5), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(17.75)
                        .turn(Math.toRadians(-90))
                        .forward(46.5 + 58.75)
                        .build();
                break;
        }

    }

    public void buildTrajectories() {

        setGoalPose();

    }

}
