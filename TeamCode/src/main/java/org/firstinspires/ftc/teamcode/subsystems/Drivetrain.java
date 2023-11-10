package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDControl;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain extends Subsystem {

    private double speedMod = 1.0;

    private DcMotorEx backRight, backLeft, frontRight, frontLeft;

    private HardwareMap hwMap;
    private Telemetry telemetry;

    private BNO055IMU imu; // Inertial measurement unit, built into expansion hub (has a gyro inside of it)
    private PIDControl pid;

    private Orientation lastAngles = new Orientation();

    double initYaw;
    double adjustedYaw;

    private double previousHeading = 0;
    private double integratedHeading = 0;

    public Drivetrain(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;
        pid = new PIDControl();

        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // This DOES NOT disable the tick counts
        // What it DOES do is disable the built in feedback loop
        // because external feedback such as our PID controller is generally preferred (and way faster)
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initializes IMU for angle control
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initYaw = lastAngles.firstAngle;
    }

    /**
     * As a last measure, if switching between auto and teleop the robot is not perfectly straight
     * You can rotate the robot in teleop, and then set the initial yaw as its current angle
     */
    public void resetInitYaw() {

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = lastAngles.firstAngle;

        // For resetting absolute angle of imu
        integratedHeading = 0;
        previousHeading = 0;

    }

    /**
     * During driver phase, if one motor lags behind, this uses PID loop to adjust
     */
    public void checkAndAdjustMotors() {

        // Say all 4 motors are running at max power (1.0)
        // Motors 3 is running at 100 rpm but motor 4 is running at 60 rpm
        // You cannot increase the power of motor 4 because it is already at max (1.0)
        // So, we check once the calculations for motor 4 are done, if the output power is greater than 1.0, we instead use motor 4 as a reference so motor 3 has to adjust to a lesser power

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        // Two front motors
        if (frontLeft.getVelocity() > frontRight.getVelocity()) {

            frontRightPower = pid.calculate(frontLeft.getVelocity(), frontRight.getVelocity(), false, false);
            if (frontRightPower > 1.0) {
                frontLeftPower = pid.calculate(frontRight.getVelocity(), frontLeft.getVelocity(), false, false);
                frontLeft.setPower(frontLeftPower);
            }else {
                frontRight.setPower(frontRightPower);
            }

        }else if (frontLeft.getVelocity() < frontRight.getVelocity()) {

            frontLeftPower = pid.calculate(frontRight.getVelocity(), frontLeft.getVelocity(), false, false);
            if (frontLeftPower > 1.0) {
                frontRightPower = pid.calculate(frontLeft.getVelocity(), frontRight.getVelocity(), false, false);
                frontRight.setPower(frontRightPower);
            }else {
                frontLeft.setPower(frontLeftPower);
            }

        }

        // Two back motors
        if (backLeft.getVelocity() > backRight.getVelocity()) {

            backRightPower = pid.calculate(backLeft.getVelocity(), backRight.getVelocity(), false, false);
            if (backRightPower > 1.0) {
                backLeftPower = pid.calculate(backRight.getVelocity(), backLeft.getVelocity(), false, false);
                backLeft.setPower(backLeftPower);
            }else {
                backRight.setPower(backRightPower);
            }

        }else if (backLeft.getVelocity() < backRight.getVelocity()) {

            backLeftPower = pid.calculate(backRight.getVelocity(), backLeft.getVelocity(), false, false);
            if (backLeftPower > 1.0) {
                backRightPower = pid.calculate(backLeft.getVelocity(), backRight.getVelocity(), false, false);
                backRight.setPower(backRightPower);
            }else {
                backLeft.setPower(backLeftPower);
            }

        }

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
        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        backLeftPower = Range.clip(backLeftPower, -1, 1);
        backRightPower = Range.clip(backRightPower, -1, 1);

        // Normalize wheel power so none exceeds 100%
        // Ensures robot maintains desired motion by keeping same ratio to every motor
        // Max = largest motor power or 1
        double max = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);

        telemetry.addData("Front Left/Right Calculated Powers", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back Left/Right Calculated Powers", "%4.2f, %4.2f", backLeftPower, backRightPower);

    }

    /**
     * Makes robot move based on a Field Oriented Drive
     * Parameters are the exact same as goXYR()
     * @param x
     * @param y
     * @param turn
     */
    public void goXYRIMU(double x, double y, double turn) {

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        adjustedYaw = lastAngles.firstAngle - initYaw;

        double zerodYaw = (-1 * initYaw) + lastAngles.firstAngle;

        double theta = Math.atan2(y, x) * 180 / Math.PI; // Angle of gamepad in degrees
        double realTheta = (360 - zerodYaw) + theta; // Real theta of robot in degrees
        double power = Math.hypot(x, y); // Power (magnitude)

        double piOverFour = Math.PI / 4.0;
        double sin = Math.sin( (realTheta * Math.PI / 180) - piOverFour);
        double cos = Math.cos( (realTheta * Math.PI / 180) - piOverFour);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = (power * cos / max + turn);
        double frontRightPower = (power * sin / max - turn);
        double backLeftPower = (power * sin / max + turn);
        double backRightPower = (power * cos / max - turn);

        // Say you drive forward at full power and turn. If the value is greater than 1, remove power from all motors to make it consistent
        if ( (power + Math.abs(turn)) > 1) {
            frontLeftPower /= power + turn;
            frontRightPower /= power - turn;
            backLeftPower /= power + turn;
            backRightPower /= power - turn;
        }

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);

        telemetry.addData("Front Left/Right Calculated Powers", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back Left/Right Calculated Powers", "%4.2f, %4.2f", backLeftPower, backRightPower);

    }

    /**
     * Strafes robot at any given angle for any magnitude of power
     **/
    public void goStrafeRadians(double angle, double magnitude) {
        double piOverFour = Math.PI / 4.0;
        double red = Math.sin(angle - piOverFour) * Range.clip(magnitude, -1, 1);
        double blue = Math.sin(angle + piOverFour) * Range.clip(magnitude, -1, 1);

        // Search up "seamonster mecanum drive" to see how math works
        frontRight.setPower(red);
        backLeft.setPower(red);
        frontLeft.setPower(blue);
        backRight.setPower(blue);
    }
    public void goStrafeDegrees(double angle, double magnitude) {
        goStrafeRadians(Math.toRadians(angle), magnitude);
    }

    public void setSpeedMod(double speedMod) {
        this.speedMod = speedMod;
    }
    public double getSpeedMod() {
        return speedMod;
    }

    /**
     * PID calibrations for motors given any position that the robot wants to move forward to
     * @param reference is the projected position in inches
     * @return
     */
    public double powerFromPIDPosition(double reference) {

        double BLcal = pid.calculate(reference, backLeft.getCurrentPosition(), false, false);
        double BRcal = pid.calculate(reference, backRight.getCurrentPosition(), false, false);
        double FLcal = pid.calculate(reference, frontLeft.getCurrentPosition(), false, false);
        double FRcal = pid.calculate(reference, frontRight.getCurrentPosition(), false, false);

        backLeft.setPower(BLcal);
        backRight.setPower(BRcal);
        frontLeft.setPower(FLcal);
        frontRight.setPower(FRcal);

        telemetry.addData("Front Left/Right Adjusted Powers", "%4.2f, %4.2f", FLcal, FRcal);
        telemetry.addData("Back Left/Right Adjusted Powers", "%4.2f, %4.2f", BLcal, BRcal);

        return 0;
    }

    public void addTelemetry() {

        telemetry.addData("IMU Absolute Angle Rotation", getIntegratedHeading());

        telemetry.addData("Front Left/Right Actual Positions", "%4.2f, %4.2f", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData("Back Left/Right Actual Positions", "%4.2f, %4.2f", backLeft.getCurrentPosition(), backRight.getCurrentPosition());


    }

    /**
     * Records the absolute angle of the imu compared to when it first started
     * @return
     */
    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }
        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }

}
