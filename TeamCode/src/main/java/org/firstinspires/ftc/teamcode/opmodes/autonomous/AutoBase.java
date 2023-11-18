package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AutoBase extends LinearOpMode {
    protected JVBoysSoccerRobot.AllianceType ALLIANCE_TYPE;
    protected PropDetectionProcessor propDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    protected VisionPortal portal;
    protected Pose2d startingPose;
    protected PropDetectionProcessor.Detection detectedSide;

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public void initialize(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot.AllianceType allianceType, Pose2d startingPose) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        ALLIANCE_TYPE = allianceType;
        this.startingPose = startingPose;

        propDetectionProcessor = new PropDetectionProcessor(ALLIANCE_TYPE);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(821.0, 821.0,330.0, 248.0) // HAVE TO SET THESE LATER
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
        telemetry.update();

        detectedSide = propDetectionProcessor.getDetectedSide();

        portal.setProcessorEnabled(aprilTagProcessor, false);
        portal.setProcessorEnabled(propDetectionProcessor, true);

        while (opModeInInit()) {
            detectedSide = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectedSide);
            telemetry.update();
            sleep(1);
        }
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
