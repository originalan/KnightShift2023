package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Axial - Driving forward and backwards - left joystick forward/backward
 * Lateral - Strafing right and left - left joystick right and left
 * Yaw - Rotating CW and CCW - right joystick right and left
 **/

@TeleOp(name = "Basic Mecanum Drive 2023", group = "Testing")

public class BasicMecanumDrive2023 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    @Override
    public void runOpMode() {
        
        /**
         * Assumes names of motors are BackLeft, BackRight, FrontLeft, FrontRight;
         **/
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        
        // May have to switch reverse and forward terms until all motors move in the forward direction irl
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        runtime.reset();
        
        if (opModeIsActive()) {
            // Put run blocks here.
            
            // Run until end of match (driver presses STOP)
            while (opModeIsActive()) {
                // Put loop blocks here.
                double max;
                
                double axial = -1 * gamepad1.left_stick_y; // pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                
                double frontLeftPower = axial + lateral + yaw;
                double frontRightPower = axial - lateral - yaw;
                double backLeftPower = axial - lateral + yaw;
                double backRightPower = axial + lateral - yaw;
                
                // Normalize wheel power so none exceeds 100%
                // Ensures robot maintains desired motion by keeping same ratio to every motor
                // Max = largest motor power or 1
                max = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
                
                // Send calculated power to wheels
                frontRight.setPower(frontRightPower);
                frontLeft.setPower(frontLeftPower);
                backRight.setPower(backRightPower);
                backLeft.setPower(backLeftPower);
                
                // Show elapsed game time and wheel power
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front Left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
                telemetry.addData("Back Left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
                
                telemetry.update();
            }
        }
        
    }
}
