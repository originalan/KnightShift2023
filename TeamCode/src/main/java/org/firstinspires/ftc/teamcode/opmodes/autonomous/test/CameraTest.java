package org.firstinspires.ftc.teamcode.opmodes.autonomous.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * CameraTest is a testing Autonomous opmode that is used to test the bounding boxes for team prop detection during the autonomous phase
 */
@Autonomous (name = "CameraTest (testing rectangles)", group = "Testing")
public class CameraTest extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private VisionPortal portal;
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;

    private AprilTagProcessor aprilTagProcessor;
    private PropDetectionProcessor propDetectionProcessor;
    private PropDetectionProcessor.Detection detectionResult;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, JVBoysSoccerRobot.AllianceType.BLUE);

        propDetectionProcessor = new PropDetectionProcessor(JVBoysSoccerRobot.AllianceType.BLUE);
        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(821.0, 821.0, 330.0, 248.0) // HAVE TO SET THESE LATER
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotSettings.CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
//                .setCamera(BuiltinCameraDirection.BACK)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessors(aprilTagProcessor, propDetectionProcessor)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        portal.setProcessorEnabled(aprilTagProcessor, false);
        portal.setProcessorEnabled(propDetectionProcessor, true);

        while (opModeInInit()) {
            propDetectionProcessor.initRectangles(JVBoysSoccerRobot.AllianceType.BLUE);
            detectionResult = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectionResult);
            telemetry.update();
            sleep(1);
        }

//        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    PropDetectionProcessor.BLUE_THRESHOLD += 0.05;
                    PropDetectionProcessor.RED_THRESHOLD += 0.05;
                }

                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    PropDetectionProcessor.BLUE_THRESHOLD -= 0.05;
                    PropDetectionProcessor.RED_THRESHOLD -= 0.05;
                }

                telemetry.addData("Prop Position", propDetectionProcessor.getDetectedSide());
                telemetry.addData("Blue threshold", "%4.3f", PropDetectionProcessor.BLUE_THRESHOLD);
                telemetry.addData("Red Threshold", "%4.3f", PropDetectionProcessor.RED_THRESHOLD);
                telemetry.update();

            }
        }
    }
}
