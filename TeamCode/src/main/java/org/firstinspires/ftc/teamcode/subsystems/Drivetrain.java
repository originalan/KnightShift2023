package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TwoDriver;
import org.firstinspires.ftc.teamcode.settings.PoseStorage;
import org.firstinspires.ftc.teamcode.settings.UseTelemetry;

/**
 * Drivetrain is a Subsystem representing all drivetrain hardware movement
 * Ex. Moving drivetrain based on Field-Centric or Robot-Centric Views during Teleop
 */
@Config
public class Drivetrain extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    private Orientation lastAngle;
    private double initYaw;
    public static double distanceThreshold = 5.0;
    public double factor = 1.0;
    public static double dSensorFactor = 3.0;

    private double  frontLeftPower,
                    frontRightPower,
                    backLeftPower,
                    backRightPower;

    public boolean orientPerpendicular = false;
    private  double  power,
            theta,
            sin,
            cos,
            max;

    public Drivetrain(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;

        lastAngle = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initYaw = PoseStorage.originalInitYaw + PoseStorage.AUTO_SHIFT_DEGREES; // b/c auto started with back facing front
    }

    @Override
    public void addTelemetry() {
        if (UseTelemetry.DRIVETRAIN) {
            telemetry.addLine("Drivetrain");

            telemetry.addData("   Front Left/Right Calculated Powers", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("   Back Left/Right Calculated Powers", "%4.2f, %4.2f", backLeftPower, backRightPower);

//            telemetry.addData("   Front Left/Right Actual Positions", "%d, %d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition());
//            telemetry.addData("   Back Left/Right Actual Positions", "%d, %d", robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
    }

    /**
     * As a last measure, if switching between auto and teleop the robot is not perfectly straight
     * You can rotate the robot in teleop, and then set the initial yaw as its current angle
     */
    public void resetInitYaw() {
        lastAngle = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = lastAngle.firstAngle;
    }

    /**
     * Moves robot based on joystick inputs and if it is field-centric or robot-centric drive
     * @param x is the strafing (left-right) motion of the robot ; left joystick left-right
     * @param y is the vertical (forward-backward) motion of the robot ; left joystick up-down
     * @param r is the rotation of the robot CW or CCW ; right joystick left-right
     * @param isFieldOriented is true if field centric drive, false if robot centric drive
     */
    public void moveXYR(double x, double y, double r, boolean isFieldOriented) {

        lastAngle = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        if (orientPerpendicular) {
//            if (Math.abs(lastAngle.firstAngle - 90) % 180 <= 1) { // if within 1 angle of perpendicular...
//                r = 0; // don't rotate
//            }
//        }

        power = 0;
        theta = 0;
        sin = 0;
        cos = 0;
        max = 0;

        if (isFieldOriented) {
            lastAngle = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double zeroedYaw = (-1 * initYaw) + lastAngle.firstAngle;
            double thetaGamepad = Math.atan2(y, x) * 180 / Math.PI; // Angle of gamepad in degrees, -180 to 180 degrees
            theta = (360 - zeroedYaw) + thetaGamepad; // Real theta that robot must travel in degrees
            theta = theta * Math.PI / 180; // convert to radians
            power = Math.hypot(x, y);
        }else {
            power = Math.hypot(x, y);
            theta = Math.atan2(y, x); // -pi to pi
        }

        if (TwoDriver.orientHelp) {

            double newTheta = Math.toDegrees(theta);
            newTheta %= 360;
            double ref = newTheta % 90;
            if (ref > 45) {
                theta = Math.toRadians( newTheta + (90 - ref) );
            }else {
                theta = Math.toRadians( newTheta - ref );
            }

        }

        sin = Math.sin(theta - (Math.PI / 4));
        cos = Math.cos(theta - (Math.PI / 4));
        max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = power * cos / max + r;
        frontRightPower = power * sin / max - r;
        backLeftPower = power * sin / max + r;
        backRightPower = power * cos / max - r;

        if (Math.abs(power) + Math.abs(r) > 1) {
            frontLeftPower /= power + Math.abs(r);
            frontRightPower /= power + Math.abs(r);
            backLeftPower /= power + Math.abs(r);
            backRightPower /= power + Math.abs(r);
        }

        robot.backLeft.setPower(backLeftPower / factor);
        robot.backRight.setPower(backRightPower / factor);
        robot.frontLeft.setPower(frontLeftPower / factor);
        robot.frontRight.setPower(frontRightPower / factor);

    }

    /**
     * Makes robot move based on x, y, r values (axial, pitch, yaw)
     *
     * Axial - Driving forward and backwards - left joystick forward/backward
     * Lateral - Strafing right and left - left joystick right and left
     * Yaw - Rotating CW and CCW - right joystick right and left
     **/
    public void goXYR(double axial, double lateral, double yaw) {
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        // Makes sure power is always between [-1, 1] just in case some goof puts in wrong values when calling this method
//        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
//        frontRightPower = Range.clip(frontRightPower, -1, 1);
//        backLeftPower = Range.clip(backLeftPower, -1, 1);
//        backRightPower = Range.clip(backRightPower, -1, 1);

        // Normalize wheel power so none exceeds 100%
        // Ensures robot maintains desired motion by keeping same ratio to every motor
        // Max = largest motor power or 1
        double max = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        robot.backLeft.setPower(backLeftPower);
        robot.backRight.setPower(backRightPower);
        robot.frontLeft.setPower(frontLeftPower);
        robot.frontRight.setPower(frontRightPower);
    }

    /**
     * Makes robot move based on a Field Oriented Drive
     * Parameters are the exact same as goXYR()
     * @param x
     * @param y
     * @param turn
     */
    public void goXYRIMU(double x, double y, double turn) {
        lastAngle = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double zerodYaw = (-1 * initYaw) + lastAngle.firstAngle;

        double theta = Math.atan2(y, x) * 180 / Math.PI; // Angle of gamepad in degrees
        double realTheta = (360 - zerodYaw) + theta; // Real theta of robot in degrees
        double power = Math.hypot(x, y); // Power (magnitude)

        double piOverFour = Math.PI / 4.0;
        double sin = Math.sin( (realTheta * Math.PI / 180) - piOverFour);
        double cos = Math.cos( (realTheta * Math.PI / 180) - piOverFour);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = (power * cos / max + turn);
        frontRightPower = (power * sin / max - turn);
        backLeftPower = (power * sin / max + turn);
        backRightPower = (power * cos / max - turn);

        // Say you drive forward at full power and turn. If the value is greater than 1, remove power from all motors to make it consistent
        if ( (power + Math.abs(turn)) > 1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power - turn;
            backLeftPower /= power + turn;
            backRightPower /= power - turn;
        }

        robot.backLeft.setPower(backLeftPower);
        robot.backRight.setPower(backRightPower);
        robot.frontLeft.setPower(frontLeftPower);
        robot.frontRight.setPower(frontRightPower);
    }

    public void dSensorCheck() {
        double left = robot.dSensorLeft.getDistance(DistanceUnit.INCH);
        double right = robot.dSensorRight.getDistance(DistanceUnit.INCH);
        telemetry.addData("dsensors are ON: ", "%.3f, %.3f", left, right);

//        // IF robot is moving 'backward', set power to 0 (y component is > 0)
//        // else, let robot move
//        if (left < 2.5 || right < 2.5) {
//            if (theta > Math.PI && theta < 2 * Math.PI) {
//                robot.backLeft.setPower(0);
//                robot.backRight.setPower(0);
//                robot.frontLeft.setPower(0);
//                robot.frontRight.setPower(0);
//            }
//        }

//        // or, we just make every power less
//        if (left < distanceThreshold || right < distanceThreshold) {
//            factor = dSensorFactor;
//        }
    }

}
