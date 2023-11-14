package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.teamcode.auto.CameraSensor;
import org.firstinspires.ftc.teamcode.auto.RedPropThreshold;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous (name = "Red1 (if you are closer to the stage)", group = "autonomous opmode")
public class Red1 extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private CameraSensor camera;
    private VisionPortal portal;
    private RedPropThreshold redPropThreshold;
    private ElapsedTime runtime = new ElapsedTime();

    private String navigation;
    private TrajectorySequence scoreSpikeMark, park;
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

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
//        camera = new CameraSensor(hardwareMap, telemetry); // openCV stuff

        redPropThreshold = new RedPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotSettings.CAMERA_NAME))
                        .setCameraResolution(new Size(640, 480))
                                .setCamera(BuiltinCameraDirection.BACK)
                                        .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        navigation = redPropThreshold.getPropPosition();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {



                telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
                telemetry.update();

            }
        }
//        robot.stop();
    }

    public void setBackdropGoalPose() {
        switch (navigation) {
            case "left":
                break;
            case "right":
                break;
            case "center":
                break;
            case "not found":
                // Should never be in this code but otherwise if it is this case, we screwed
                break;
        }
    }

    public void buildTrajectories() {



    }

}
