package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;

@TeleOp(name = "Red1_3T_2P", group = "Autonomous Opmode 11.19")
public class Red1_3T_2P extends AutoBase{

    private TrajectorySequence traj1;

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

            drive.followTrajectorySequence(traj1);

        }

        transferPose();

    }

    /**
     * Determines path based on location of prop
     */
    public void setGoalPose() {



    }

    public void buildTrajectories() {

        traj1 = drive.trajectorySequenceBuilder(startingPose)
                .waitSeconds(1)
                .forward(26.25)
                .turn(Math.toRadians(-90))
                .forward(38)
                .build();

    }

}
