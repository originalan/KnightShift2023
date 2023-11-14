package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.PIDControl;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;

@TeleOp(name = "OneDriver", group = "teleopmode")

public class OneDriver extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;

    private boolean intakeOn = false;
    private boolean moveRigServo = false;
    private boolean switchDriveControls = false;


    @Override
    public void runOpMode() { // Remember that 'hardwareMap' is only visible in this method, not class
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        double reversed = 1.0;

//        PIDControl pid = new PIDControl(hardwareMap, telemetry);
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        // WAIT FOR THIS TELEMETRY MESSAGE BEFORE PRESSING START because IMU takes a while to be initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Put loop blocks here.

                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Records joystick values
//                double axial = -1 * gamepad1.left_stick_y * reversed; // pushing stick forward gives negative value
//                double lateral = gamepad1.left_stick_x * reversed;
//                double yaw = gamepad1.right_stick_x * reversed;

                // You have to take values as if the robot is 90 degrees rotated because the expansion hub is placed 90 degrees rotated
                // I rotated (x,y) 90 degrees CW to (y, -x)
                double axialIMU = -1 * gamepad1.left_stick_x * reversed;
                double lateralIMU = -1 * gamepad1.left_stick_y * reversed;
                double yawIMU = gamepad1.right_stick_x * reversed;

                // Prem said this is good cuz you can easily see what is wrong if robot strafes off
                double axialIMU2 = gamepad1.left_stick_x * reversed;
                double lateralIMU2 = -1 * gamepad1.right_stick_y * reversed;
                double yawIMU2 = (gamepad1.right_trigger - gamepad1.left_trigger) * reversed;

                // Moves drivetrain based on joystick values and updates telemetry with wheel powers
                // drivetrain.goXYR(axial, lateral, yaw, telemetry);

                // Moves drivetrain on a field orientated drive and updates telemetry with wheel powers
                if (switchDriveControls) {
                    robot.drivetrain.goXYRIMU(axialIMU2, lateralIMU2, yawIMU2);
                }else {
                    robot.drivetrain.goXYRIMU(axialIMU, lateralIMU, yawIMU);
                }

                // PID control that adjusts for any irl inconsistencies with motor velocity
                // drivetrain.checkAndAdjustMotors();

                // Show elapsed game time
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                robot.addTelemetry();

//                if (currentGamepad1.a && !previousGamepad1.a) {
//                    // changes back of robot to front (controls are based on front of robot)
//                    reversed = (reversed == -1.0 ? 1.0 : -1.0);
//                }

                // For debugging
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    switchDriveControls = !switchDriveControls;
                }

                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.drivetrain.resetInitYaw();
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    moveRigServo = !moveRigServo;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    intakeOn = !intakeOn;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                }

                if (moveRigServo) {
                    robot.rig.riggingState = Rigging.RiggingState.RIG;
                }else {
                    robot.rig.riggingState = Rigging.RiggingState.NO_RIG;
                }

                if (intakeOn) {
                    robot.intake.intakeState = Intake.IntakeState.ON;
                }else {
                    robot.intake.intakeState = Intake.IntakeState.OFF;
                }

                // Update all subsystems (if applicable since drivetrain needs no update)
                robot.update();

                telemetry.update();
            }
        }
        robot.stop();
    }

}
