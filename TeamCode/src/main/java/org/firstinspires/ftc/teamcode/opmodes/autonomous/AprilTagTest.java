package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "April Tag Test", group = "Testing")
public class AprilTagTest extends AutoBase {
    @Override
    public void runOpMode() {

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (aprilTagProcessor.getDetections().size() > 0) {
                    AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);

                    telemetry.addData("Amount of Tags detected", aprilTagProcessor.getDetections().size());

                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Tag center point", "(%.3f, %.3f)", tag.center.x, tag.center.y);
                    telemetry.addData("Tag X, Y, Z", "%.3f  %.3f  %.3f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
                    telemetry.addData("Tag roll, pitch, yaw", "%.3f  %.3f  %.3f", tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw);
                    telemetry.addData("Tag elevation", "%.3f",  tag.ftcPose.elevation);
                    telemetry.addData("Tag bearing", "%.3f", tag.ftcPose.bearing);
                    telemetry.addData("Tag range", "%.3f", tag.ftcPose.range);
                }

                telemetry.update();

            }
        }

    }


}
