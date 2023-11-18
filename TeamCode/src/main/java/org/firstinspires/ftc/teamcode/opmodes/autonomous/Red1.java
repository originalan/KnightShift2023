package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.teamcode.auto.CameraSensor;
import org.firstinspires.ftc.teamcode.auto.RedPropThreshold;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "Red1 (if you are closer to the stage)", group = "Autonomous Opmode")
public class Red1 extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private VisionPortal portal;
    private ElapsedTime runtime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private boolean atBackdrop, runningTrajectory;


    private Pose2d
            spikeMarkGoalPose,
            initialBackdropGoalPose,
            firstStackPose,
            firstCycleBackdropGoalPose;

    private final long // in milliseconds
        BACKDROP_WAIT_TIME = 0,
        SCANNING_TIME = 1000,
        SCORE_WAIT_TIME = 500,
        INTAKING_TIME = 2000;

    // TODO: adjust this for each auto
    private Pose2d startPose = new Pose2d(11.75,-70.5 + (RobotSettings.ROBOT_SIDE_LENGTH / 2), Math.toRadians(90));
    private TrajectorySequence scoreSpikeMark, getStackPixels, adjustStack, getStackPixels2, scoreFirstStackPixels, park;

    private AprilTagProcessor aprilTagProcessor;
    private PropDetectionProcessor propDetectionProcessor;
    private PropDetectionProcessor.Detection detectionResult;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry, JVBoysSoccerRobot.AllianceType.RED);

        propDetectionProcessor = new PropDetectionProcessor(JVBoysSoccerRobot.AllianceType.RED);
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
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        portal.setProcessorEnabled(aprilTagProcessor, false);
        portal.setProcessorEnabled(propDetectionProcessor, true);

        while (opModeInInit()) {
            detectionResult = propDetectionProcessor.getDetectedSide();
            telemetry.addData("Detected", detectionResult);
            telemetry.update();
            sleep(1);
        }

//        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

//        setBackdropGoalPose();
//        buildTrajectories();

//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(-35.25, -11.75), Math.toRadians(0))
//                .build();
//
//        drive.followTrajectory(traj1);

        boolean changeValue = false;
        double elapsedTimeSinceButton = -1;
        double currentTime = -1;

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.x && !previousGamepad1.x) {

                    propDetectionProcessor.BLUE_THRESHOLD = propDetectionProcessor.BLUE_THRESHOLD + 0.1;
                    propDetectionProcessor.RED_THRESHOLD = propDetectionProcessor.RED_THRESHOLD + 0.1;

                }

                if (currentGamepad1.a && !previousGamepad1.a) {

                    propDetectionProcessor.BLUE_THRESHOLD = propDetectionProcessor.BLUE_THRESHOLD - 0.1;
                    propDetectionProcessor.RED_THRESHOLD = propDetectionProcessor.RED_THRESHOLD - 0.1;

                }

                telemetry.addData("Prop Position", propDetectionProcessor.getDetectedSide());
                telemetry.addData("Blue threshold", "%4.3f", propDetectionProcessor.BLUE_THRESHOLD);
                telemetry.addData("Red Threshold", "%4.3f", propDetectionProcessor.RED_THRESHOLD);
                telemetry.update();

            }
        }
//        robot.stop();
    }

    public void buildTrajectories() {

        //

    }

}
