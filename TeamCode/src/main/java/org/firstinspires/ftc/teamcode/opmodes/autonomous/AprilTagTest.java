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

                    telemetry.addData("Tag X", tag.ftcPose.x);
                    telemetry.addData("Tag Y", tag.ftcPose.y);
                    telemetry.addData("Tag Z", tag.ftcPose.z);
                    telemetry.addData("Roll", tag.ftcPose.roll);
                    telemetry.addData("Pitch", tag.ftcPose.pitch);
                    telemetry.addData("Yaw", tag.ftcPose.yaw);
                }

                telemetry.update();

            }
        }

    }


}
