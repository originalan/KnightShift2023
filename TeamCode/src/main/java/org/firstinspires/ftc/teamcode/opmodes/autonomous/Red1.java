package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.teamcode.RobotSettings;
import org.firstinspires.ftc.teamcode.auto.CameraSensor;
import org.firstinspires.ftc.teamcode.auto.RedPropThreshold;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Autonomous (name = "Red1 (if you are closer to the stage)")
public class Red1 extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private CameraSensor camera;
    private VisionPortal portal;
    private RedPropThreshold redPropThreshold;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        camera = new CameraSensor(hardwareMap, telemetry);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, RobotSettings.CAMERA_NAME))
                        .setCameraResolution(new Size(640, 480))
                                .setCamera(BuiltinCameraDirection.BACK)
                                        .build();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
                telemetry.update();

            }
        }
        robot.stop();
    }

}
