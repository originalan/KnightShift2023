package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while (opModeInInit()) {
            drive.raiseArm(); // added for our specific robot
        }

        waitForStart();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(24, 24), 0)
                .waitSeconds(2)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {
            drive.raiseArm(); // added for our specific robot
            drive.update();
        }

    }
}
