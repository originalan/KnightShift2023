package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import com.qualcomm.robotcore.util.Range;

/**
 * Drivetrain is a Subsystem representing all drivetrain hardware movement
 * Ex. Moving drivetrain based on Field-Centric or Robot-Centric Views during Teleop
 */
public class Drivetrain extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    private Orientation lastAngles;
    private double initYaw;

    private double  frontLeftPower,
                    frontRightPower,
                    backLeftPower,
                    backRightPower;

    public boolean orientPerpendicular = false;

    public Drivetrain(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;

        lastAngles = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // We want robot to face 90 degrees (in pose2d graph and in radians)
        // insert some code here so that if robot ends up not facing this after auto, adjust initYaw to something that works
        double goalHeading = PoseStorage.startingAutoPose.getHeading();
        goalHeading = Math.toDegrees(goalHeading);

        double curHeading = PoseStorage.currentPose.getHeading();
        curHeading = Math.toDegrees(curHeading);

        double diff = goalHeading - curHeading;

        initYaw = lastAngles.firstAngle + diff;
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Drivetrain");

        telemetry.addData("   Front Left/Right Calculated Powers", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("   Back Left/Right Calculated Powers", "%4.2f, %4.2f", backLeftPower, backRightPower);

        telemetry.addData("   Front Left/Right Actual Positions", "%d, %d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition());
        telemetry.addData("   Back Left/Right Actual Positions", "%d, %d", robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
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
        lastAngles = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = lastAngles.firstAngle;

        // For resetting absolute angle of imu (purely for telemetry purposes)
        robot.resetIMUAngle();
    }

    /**
     * Moves robot based on joystick inputs and if it is field-centric or robot-centric drive
     * @param x is the strafing (left-right) motion of the robot ; left joystick left-rightx
     * @param y is the vertical (forward-backward) motion of the robot ; left joystick up-down
     * @param r is the rotation of the robot CW or CCW ; right joystick left-right
     * @param isFieldOriented is true if field centric drive, false if robot centric drive
     */
    public void moveXYR(double x, double y, double r, boolean isFieldOriented) {

        lastAngles = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (orientPerpendicular) {
            if (Math.abs(lastAngles.firstAngle - 90) % 180 <= 1) { // if within 1 angle of perpendicular...
                r = 0; // don't rotate
            }
        }

        double  power,
                theta,
                sin,
                cos,
                max;

        if (isFieldOriented) {
            lastAngles = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double zeroedYaw = (-1 * initYaw) + lastAngles.firstAngle;
            double thetaGamepad = Math.atan2(y, x) * 180 / Math.PI; // Angle of gamepad in degrees
            theta = (360 - zeroedYaw) + thetaGamepad; // Real theta that robot must travel in degrees
            theta = theta * Math.PI / 180; // convert to radians
            power = Math.hypot(x, y);
        }else {
            power = Math.hypot(x, y);
            theta = Math.atan2(y, x);
        }

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = power * cos / max + r;
        frontRightPower = power * sin / max - r;
        backLeftPower = power * sin / max + r;
        backRightPower = power * cos / max - r;

        if ((power + Math.abs(r) > 1)) {
            frontLeftPower /= power + Math.abs(r);
            frontRightPower /= power + Math.abs(r);
            backLeftPower /= power + Math.abs(r);
            backRightPower /= power + Math.abs(r);
        }

        robot.backLeft.setPower(backLeftPower);
        robot.backRight.setPower(backRightPower);
        robot.frontLeft.setPower(frontLeftPower);
        robot.frontRight.setPower(frontRightPower);

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
        lastAngles = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double zerodYaw = (-1 * initYaw) + lastAngles.firstAngle;

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

    /**
     * Uses IMU to return angle of robot
     * @return
     */
    public double getAngle() {
        Orientation angles = robot.imu2.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

}
