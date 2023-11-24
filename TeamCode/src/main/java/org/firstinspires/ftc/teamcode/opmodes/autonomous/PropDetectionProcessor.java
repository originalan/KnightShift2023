package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class PropDetectionProcessor implements VisionProcessor {

    private final JVBoysSoccerRobot.AllianceType ALLIANCE_TYPE;

    // Mats
    private final Mat testMat = new Mat();
    private final Mat lowMat = new Mat();
    private final Mat highMat = new Mat();
    private final Mat finalMat = new Mat();

    // Rectangles
    public Rect LEFT_RECTANGLE;
    public Rect RIGHT_RECTANGLE;

    public static Rect leftRectRed = new Rect(
            // top left corner
            new Point(50, 120),
            // bottom right corner
            new Point(235, 275)
    );
    public static Rect leftRectBlue = new Rect(
            new Point(50, 120),
            // bottom right corner
            new Point(235, 275)
    );
    public static Rect rightRectRed = new Rect(
            new Point(470, 120),
            new Point(640, 240)
    );
    public static Rect rightRectBlue = new Rect(
            new Point(470, 120),
            new Point(640, 240)
    );

    // Red constants
    public static double RED_THRESHOLD = 0.3;
    private final Scalar LOW_HSV_RED_LOWER = new Scalar(0, 100, 20);
    private final Scalar LOW_HSV_RED_UPPER = new Scalar(10, 255, 255);
    private final Scalar HIGH_HSV_RED_LOWER = new Scalar(160, 100, 20);
    private final Scalar HIGH_HSV_RED_UPPER = new Scalar(180, 255, 255);

    // Blue constants
    public static double BLUE_THRESHOLD = 0.3;
    private final Scalar LOW_HSV_BLUE_LOWER = new Scalar(100, 100, 20);
    private final Scalar LOW_HSV_BLUE_UPPER = new Scalar(130, 255, 255);
    private final Scalar HIGH_HSV_BLUE_LOWER = new Scalar(220, 100, 20);
    private final Scalar HIGH_HSV_BLUE_UPPER = new Scalar(250, 255, 255);

    public enum Detection {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public PropDetectionProcessor(JVBoysSoccerRobot.AllianceType type) {
        ALLIANCE_TYPE = type;

        // Init rectangles
        // Red
        initRectangles(type);
    }

    public void initRectangles(JVBoysSoccerRobot.AllianceType type) {

        // left rectangle bounding boxes
        LEFT_RECTANGLE = (type == JVBoysSoccerRobot.AllianceType.RED) ? leftRectRed : leftRectBlue;

        // right rectangle bounding boxes
        RIGHT_RECTANGLE = (type == JVBoysSoccerRobot.AllianceType.RED) ? rightRectRed : rightRectBlue;
    }

    private Detection detectedSide = Detection.RIGHT;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if (this.ALLIANCE_TYPE == JVBoysSoccerRobot.AllianceType.RED) {
            Core.inRange(testMat, LOW_HSV_RED_LOWER, LOW_HSV_RED_UPPER, lowMat);
            Core.inRange(testMat, HIGH_HSV_RED_LOWER, HIGH_HSV_RED_UPPER, highMat);
        } else {
            Core.inRange(testMat, LOW_HSV_BLUE_LOWER, LOW_HSV_BLUE_UPPER, lowMat);
            Core.inRange(testMat, HIGH_HSV_BLUE_LOWER, HIGH_HSV_BLUE_UPPER, highMat);
        }

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;

        double allianceBasedThreshold = ALLIANCE_TYPE == JVBoysSoccerRobot.AllianceType.RED ? RED_THRESHOLD : BLUE_THRESHOLD;

        if (averagedLeftBox > allianceBasedThreshold) {
            detectedSide = Detection.LEFT;
        } else if (averagedRightBox > allianceBasedThreshold){
            detectedSide = Detection.MIDDLE;
        } else {
            detectedSide = Detection.RIGHT;
        }

        // comment out during competition
        // Line should only be added in when you want to see your custom pipeline on the driver station stream
        finalMat.copyTo(frame);

        return null; // return null b/c you do not return the original mat anymore
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(2);

        RectF leftRect = new RectF(LEFT_RECTANGLE.x * scaleBmpPxToCanvasPx, LEFT_RECTANGLE.y * scaleBmpPxToCanvasPx, (LEFT_RECTANGLE.x + LEFT_RECTANGLE.width) * scaleBmpPxToCanvasPx, (LEFT_RECTANGLE.y + LEFT_RECTANGLE.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(leftRect, paint);

        paint.setColor(Color.BLUE);
        RectF rightRect = new RectF(RIGHT_RECTANGLE.x * scaleBmpPxToCanvasPx, RIGHT_RECTANGLE.y * scaleBmpPxToCanvasPx, (RIGHT_RECTANGLE.x + RIGHT_RECTANGLE.width) * scaleBmpPxToCanvasPx, (RIGHT_RECTANGLE.y + RIGHT_RECTANGLE.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(rightRect, paint);

    }

    public Detection getDetectedSide() {
        return detectedSide;
    }

}
