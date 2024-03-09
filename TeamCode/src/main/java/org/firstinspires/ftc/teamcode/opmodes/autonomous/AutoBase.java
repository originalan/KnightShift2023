package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.PoseStorage;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * AutoBase is the base used for all autonomous opmodes
 * Streamlines the creation of multiple autonomous opmodes with different paths
 */
@Config
public abstract class AutoBase extends LinearOpMode {
    public static double CLAW_MIDDLE_FROM_FRONT = 2.25;
    public static double CLAW_MIDDLE_FROM_CENTERLINE = 1.725;

    protected ElapsedTime runtime = new ElapsedTime();
    protected boolean checkAgain = true;
    protected JVBoysSoccerRobot.AllianceType ALLIANCE_TYPE;
    protected PropDetectionProcessor propDetectionProcessor;
    protected AprilTagProcessor aprilTagProcessor;
    protected VisionPortal portal;
    protected Pose2d startingPose;
    protected PropDetectionProcessor.Detection detectedSide;
    protected PropDetectionProcessor.Detection previousDetectedSide;
    protected JVBoysSoccerRobot robot;
    protected BulkReading bulkReading;
    protected SampleMecanumDrive drive;
    protected final Pose2d redCloseStart = new Pose2d(12, -54.3, Math.toRadians(90));
    protected final Pose2d redFarStart = new Pose2d(-36, -54.3, Math.toRadians(90));
    protected final Pose2d blueCloseStart = new Pose2d(12, 54.3, Math.toRadians(270));
    protected final Pose2d blueFarStart = new Pose2d(-36, 54.3, Math.toRadians(270));
    protected Gamepad currentGamepad = new Gamepad();
    protected Gamepad previousGamepad = new Gamepad();

    public void initialize(JVBoysSoccerRobot.AllianceType allianceType) {


        ALLIANCE_TYPE = allianceType;
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, ALLIANCE_TYPE);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        PoseStorage.originalInitYaw = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        propDetectionProcessor = new PropDetectionProcessor(ALLIANCE_TYPE);
        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(829.841, 829.841,323.788, 251.973)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                // or focalLength="822.317f, 822.317f"
                //            principalPoint="319.495f, 242.502f" (from default camera calibration in xml file)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTagProcessor.setDecimation(3);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotSettings.CAMERA_NAME))
                .setCameraResolution(new Size(640, 480))
//                .setCamera(BuiltinCameraDirection.BACK)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessors(aprilTagProcessor, propDetectionProcessor)
                .build();

        portal.setProcessorEnabled(aprilTagProcessor, false);
        portal.setProcessorEnabled(propDetectionProcessor, true);

        detectedSide = propDetectionProcessor.getDetectedSide();
        previousDetectedSide = propDetectionProcessor.copyDetection(detectedSide);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());

    }

    public void transferPose() {

        PoseStorage.currentPose = drive.getPoseEstimate();

        telemetry.addData("Pose X", PoseStorage.currentPose.getX());
        telemetry.addData("Pose Y", PoseStorage.currentPose.getY());
        telemetry.addData("Pose Heading", PoseStorage.currentPose.getHeading());

    }

    public void dSensorAdjust() {

        double left = robot.dSensorLeft.getDistance(DistanceUnit.INCH);
        double right = robot.dSensorRight.getDistance(DistanceUnit.INCH);
        // distance of 9.4 inches between distance sensors

        double t;

        if (left - right > 0) {
            t = Math.atan2(left - right, 9.4);
            // robot needs to turn counterclockwise, positive turn value
        }else {
            t = Math.atan2(right - left, 9.4) * -1;
            // robot needs to turn clockwise
        }
        drive.turn(t);

    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
