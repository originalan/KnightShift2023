package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous (name = "April Tag Robot Test", group = "Testing")
public class AprilTagRobotTest extends AutoBase {

    public static double distance = 5;
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

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad2.copy(gamepad2);

                if (currentGamepad1.x && !previousGamepad1.x) {

                    if (aprilTagProcessor.getDetections().size() > 0) {
                        AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);

                        double range = tag.ftcPose.range;
                        double bearing = tag.ftcPose.bearing;
                        double yaw = tag.ftcPose.yaw;

                        double x = range * Math.sin(Math.toRadians(bearing));
                        double y = range * Math.cos(Math.toRadians(bearing));
                        startingPose = new Pose2d(x, y, Math.toRadians(yaw));
                        drive.setPoseEstimate(startingPose);

                        trajectory = drive.trajectorySequenceBuilder(startingPose)
                                .turn(-yaw)
                                .lineTo(new Vector2d(0, distance))
                                .build();

                        drive.followTrajectorySequence(trajectory);
                    }
                }

                telemetry.update();
            }
        }
    }
}
