package org.firstinspires.ftc.teamcode.opmodes.autonomous.old.red;

import static org.firstinspires.ftc.teamcode.util.RobotSettings.AUTO_PURPLE_PIXEL_RELEASE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@Autonomous(name = "Red1_2T_1P (places pixel, parks outer", group = "Autonomous Opmode 11.19")
public class Red1_2T_1P extends AutoBase {

    private TrajectorySequence traj1;
    private TrajectorySequence detectionTraj;

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
                        .splineTo(new Vector2d(11.75 - 1.5, -22.0), Math.toRadians(90))
                        // Center of robot, adjusted so purple pixel servo is in line with the center
                        .UNSTABLE_addDisplacementMarkerOffset(AUTO_PURPLE_PIXEL_RELEASE, () -> {
                            robot.purplePixel.drop();
                        })
                        .back(36.25)
                        .turn(Math.toRadians(-90))
                        .forward(47)
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
                        .turn(Math.toRadians(-90))
                        .forward(47)
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
                        .turn(Math.toRadians(-90))
                        .forward(35.25)
                        .build();
                break;
        }

    }

    public void buildTrajectories() {

        setGoalPose();

    }

}
