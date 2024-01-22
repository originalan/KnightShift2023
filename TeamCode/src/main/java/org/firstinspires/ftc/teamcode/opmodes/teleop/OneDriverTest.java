package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * OneDriverTest is identical to OneDriver except it has more joystick controls for testing certain hardware.
 */
@TeleOp(name = "OneDriverTest", group = "Testing")
public class OneDriverTest extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private boolean launcherFired = false;
    private boolean isRigging = false;
    private boolean rigStringMove = false;
    private boolean switchDriveControls = false;
    private boolean positionArm = false;


    @Override
    public void runOpMode() {
        telemetry.addLine("WAIT FOR INITIALIZATION MESSAGE BEFORE PRESSING START");
        telemetry.update();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        // WAIT FOR THIS TELEMETRY MESSAGE BEFORE PRESSING START because IMU takes a while to be initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        while (opModeInInit()) {
            robot.launcher.launcherState = AirplaneLauncher.LauncherState.AT_REST;
            telemetry.addLine("AT REST");
        }

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Records joystick values
                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y * -1;
                double r = gamepad1.right_stick_x;

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    switchDriveControls = !switchDriveControls;
                }

                robot.drivetrain.moveXYR(x, y, r, !switchDriveControls);

                // Show elapsed game time
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                // Add all robot telemetry
                robot.addTelemetry();

                /*
                =================DELIVERY ARM CONTROLS==============
                */
                if (currentGamepad1.y && !previousGamepad1.y) {
                    positionArm = !positionArm;
                }
                if (positionArm) {
                    robot.deliveryArm.armState = DeliveryArm.ArmState.TOP;
                    robot.intake.intakeState = Intake.IntakeState.HOLDING_PIXEL;
                    // Because intake controls are underneath this if statement, we can override this state if we activate the intake
                }else {
                    robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.deliveryArm.armState = DeliveryArm.ArmState.BOTTOM;
                }

                // Manual "override" of PIDF control of delivery arm
                if (currentGamepad1.left_bumper) {
                    robot.deliveryArm.overridePowerForward = true;
                }else {
                    robot.deliveryArm.overridePowerForward = false;
                }

                if (currentGamepad1.right_bumper) {
                    robot.deliveryArm.overridePowerBackward = true;
                }else {
                    robot.deliveryArm.overridePowerBackward = false;
                }

                /*
                =================INTAKE CONTROLS==============
                */
                if (Math.abs(currentGamepad1.left_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.FORWARD;
                }else if (Math.abs(currentGamepad1.right_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.REVERSE;
                }else {
                    robot.intake.intakeState = Intake.IntakeState.OFF;
                }

                /*
                =================RIGGING CONTROLS==============
                */

                if (currentGamepad1.x && !previousGamepad1.x) {
                    isRigging = !isRigging;
                }

                if (!rigStringMove) {
                    if (isRigging) {
                        robot.rightRigServo.getController().pwmEnable();
                        robot.leftRigServo.getController().pwmEnable();
                        robot.rig.hang();
                    }else {
                        robot.rightRigServo.getController().pwmDisable();
                        robot.leftRigServo.getController().pwmDisable();
                    }
                }else {
                    robot.rightRigServo.getController().pwmDisable();
                    robot.leftRigServo.getController().pwmDisable();
                }

                if (isRigging && (currentGamepad1.dpad_right || currentGamepad1.dpad_left)) {
                    rigStringMove = true;
                    robot.rightRigMotor.setPower(-1 * RobotSettings.RIGGING_MOTOR_SPEED);
                    robot.leftRigMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                }
                else {
                    robot.rightRigMotor.setPower(0);
                    robot.leftRigMotor.setPower(0);
                }

                /*
                =================AIRPLANE CONTROLS==============
                */
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    launcherFired = !launcherFired;
                }

                if (launcherFired) {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.ZONE_ONE_OR_BUST;
                    telemetry.addLine("LAUNCHER FIRED");
                }else {
                    robot.launcher.launcherState = AirplaneLauncher.LauncherState.AT_REST;
                    telemetry.addLine("LAUNCHER AT REST");
                }

                /*
                =================FAILSAFE FIELD-ORIENTED VIEW CONTROLS==============
                */
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.drivetrain.resetInitYaw();
                }

                /*
                =================TOGGLE UTIL CONTROLS==============
                */
                if (currentGamepad1.a && !previousGamepad1.a) { // never tested this lol
                    robot.drivetrain.orientPerpendicular = !robot.drivetrain.orientPerpendicular;
                }

                // Update all subsystems (if applicable)
                robot.update();
                telemetry.update();
            }
        }
    }

}
