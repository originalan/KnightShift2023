package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * AutoBase is the base used for all autonomous opmodes
 * Streamlines the creation of multiple autonomous opmodes with different paths
 */
public abstract class AutoBase extends LinearOpMode {
    protected JVBoysSoccerRobot.AllianceType ALLIANCE_TYPE;
    protected PropDetectionProcessor propDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    protected VisionPortal portal;
    protected Pose2d startingPose;
    protected PropDetectionProcessor.Detection detectedSide;
    protected JVBoysSoccerRobot robot;
    protected SampleMecanumDrive drive;

    public void initialize(JVBoysSoccerRobot.AllianceType allianceType) {
        ALLIANCE_TYPE = allianceType;
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, ALLIANCE_TYPE);

        PoseStorage.originalInitYaw = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        propDetectionProcessor = new PropDetectionProcessor(ALLIANCE_TYPE);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(829.841, 829.841,323.788, 251.973)
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

        portal.setProcessorEnabled(aprilTagProcessor, false);
        portal.setProcessorEnabled(propDetectionProcessor, true);

        detectedSide = propDetectionProcessor.getDetectedSide();

    }

    public void transferPose() {

        PoseStorage.currentPose = drive.getPoseEstimate();

        telemetry.addData("Pose X", PoseStorage.currentPose.getX());
        telemetry.addData("Pose Y", PoseStorage.currentPose.getY());
        telemetry.addData("Pose Heading", PoseStorage.currentPose.getHeading());

    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
