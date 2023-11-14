package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraSensor {

    private OpenCvWebcam webcam;
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private AprilTagDetectionPipeline pipeline1;

    public CameraSensor(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        // aasdfasd
    }

    public void initCamera() {

        int cameraMonitorViewId = hwMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hwMap.get(WebcamName.class, RobotSettings.CAMERA_NAME), cameraMonitorViewId);
        // webcam.setPipeline();

        webcam.setMillisecondsPermissionTimeout(2500);

        // Streaming Frames
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }
                    @Override
                    public void onError(int errorCode) { }
                }
        );

    }

}
