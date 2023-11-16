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
    private CameraSensor camera;
    private VisionPortal portal;
    private RedPropThreshold redPropThreshold;
    private ElapsedTime runtime = new ElapsedTime();

    private String navigation;

    private enum EXAMPLE {
        case1,
        case2
    };
    private EXAMPLE ex = EXAMPLE.case1;
    private SampleMecanumDrive drive;
    private boolean atBackdrop, runningTrajectory;

    // all spike mark locations since I'm lazy
    private Pose2d redLeftSideLeftSpikeMark = new Pose2d(-47.5,-36);
    private Pose2d redLeftSideMiddleSpikeMark = new Pose2d(-36,-24.5);
    private Pose2d redLeftSideRightSpikeMark = new Pose2d(-24.5,-36);
    private Pose2d redRightSideLeftSpikeMark = new Pose2d(0.5,-36);
    private Pose2d redRightSideMiddleSpikeMark = new Pose2d(12,-24.5);
    private Pose2d redRightSideRightSpikeMark = new Pose2d(23.5,-36);
    private Pose2d blueLeftSideLeftSpikeMark = new Pose2d(23.5,36);
    private Pose2d blueLeftSideMiddleSpikeMark = new Pose2d(12,24.5);
    private Pose2d blueLeftSideRightSpikeMark = new Pose2d(0.5,36);
    private Pose2d blueRightSideLeftSpikeMark = new Pose2d(-24.5,36);
    private Pose2d blueRightSideMiddleSpikeMark = new Pose2d(-36,24.5);
    private Pose2d blueRightSideRightSpikeMark = new Pose2d(-47.5,36);

    // backdrop april tag locations
    private Pose2d blueLeftBackdrop = new Pose2d(60.75, 72-22.5-6.625);
    private Pose2d blueMiddleBackdrop = new Pose2d(60.75, 72-22.5-12.75);
    private Pose2d blueRightBackdrop = new Pose2d(60.75, 72-22.5-18.75);
    private Pose2d redLeftBackdrop = new Pose2d(60.75, -72+22.5+18.75);
    private Pose2d redMiddleBackdrop = new Pose2d(60.75, -72+22.5+12.75);
    private Pose2d redRightBackdrop = new Pose2d(60.75, -72+22.5+6.625);

    // white pixel stack locations
    private Pose2d redOuterStack = new Pose2d(-72, -72+36);
    private Pose2d redMiddleStack = new Pose2d(-72, -72+48);
    private Pose2d redInnerStack = new Pose2d(-72, -72+60);
    private Pose2d blueInnerStack = new Pose2d(-72, 72-60);
    private Pose2d blueMiddleStack = new Pose2d(-72, 72-48);
    private Pose2d blueOuterStack = new Pose2d(-72, 72-36);

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
            sleep(10);
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

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);



                telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
                telemetry.addData("Redthreshold", "%4.3f", redPropThreshold.redThreshold);
                telemetry.update();

            }
        }
//        robot.stop();
    }
//
//    public void setBackdropGoalPose() {
//        switch (navigation) {
//            case "left":
//
//                spikeMarkGoalPose = new Pose2d(redRightSideLeftSpikeMark.getX()+(ROBOT_FRONT_LENGTH/Math.sqrt(2)), redRightSideLeftSpikeMark.getY()-(ROBOT_FRONT_LENGTH/Math.sqrt(2)), Math.toRadians(135));
//                initialBackdropGoalPose = new Pose2d(redLeftBackdrop.getX()-ROBOT_BACK_LENGTH -0.5, 2.5-2.5+redLeftBackdrop.getY() -0.5, Math.toRadians(180));
//                firstCycleBackdropGoalPose = new Pose2d(redMiddleBackdrop.getX()-ROBOT_BACK_LENGTH+0.5 -0.5, -1.5-2.5+redMiddleBackdrop.getY(), Math.toRadians(180));
//                firstStackPose = new Pose2d(redInnerStack.getX()+ROBOT_FRONT_LENGTH+ROBOT_INTAKE_LENGTH-0.5, redInnerStack.getY()-3);
//
//                break;
//            case "right":
//                break;
//            case "center":
//                break;
//            case "not found":
//                // Should never be in this code but otherwise if it is this case, we screwed or i'll change the default to "left"
//                break;
//        }
//    }

    public void buildTrajectories() {

        // this does the scoring on the spike mark at the start of auto
        scoreSpikeMark = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(new Vector2d(12,-48), Math.toRadians(90))
                .splineToSplineHeading(spikeMarkGoalPose, spikeMarkGoalPose.getHeading())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(15,-48), Math.toRadians(90))
//                .UNSTABLE_addTemporalMarkerOffset(0,()-> twoPersonDrive.startPreset(0, false))
                .splineToSplineHeading(new Pose2d(30, -56, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, initialBackdropGoalPose.getY(), Math.toRadians(180.00001)), Math.toRadians(90))
                .setReversed(false)
                .lineToLinearHeading(initialBackdropGoalPose)
                .resetConstraints()
                .build();



    }

}
