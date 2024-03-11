package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous (name = "April Tag Robot Test", group = "Testing")
public class AprilTagRobotTest extends AutoBase {

    public static double distance = 7.5;
    public static int TAG_ID = 5;
    private TrajectorySequence trajectory;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        portal.setProcessorEnabled(propDetectionProcessor, false);
        portal.setProcessorEnabled(aprilTagProcessor, true);

        telemetry.addLine("Distance = " + distance + " inches away from camera");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                if (currentGamepad1.x && !previousGamepad1.x) {

                    if (aprilTagProcessor.getDetections().size() > 0) {
                        for (AprilTagDetection tag : aprilTagProcessor.getDetections()) {
                            if (tag.id == TAG_ID) {
                                double yaw = tag.ftcPose.yaw;

                                double angle1 = tag.ftcPose.bearing - tag.ftcPose.yaw;
                                double x = (tag.ftcPose.range * Math.cos(Math.toRadians(angle1))) + (6.27 * Math.sin( Math.toRadians(tag.ftcPose.yaw) ));
                                double y = (tag.ftcPose.range * Math.sin(Math.toRadians(angle1))) + (6.27 * Math.cos( Math.toRadians(tag.ftcPose.yaw) ));
                                startingPose = new Pose2d(x, y, Math.toRadians(180 - yaw));
                                drive.setPoseEstimate(startingPose);

                                trajectory = drive.trajectorySequenceBuilder(startingPose)
                                        .turn(yaw)
                                        .lineTo(new Vector2d(distance, 0))
                                        .build();

                                drive.followTrajectorySequence(trajectory);
                            }
                        }
                    }
                }

                telemetry.update();
            }
        }
    }
}
