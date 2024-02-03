package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * IntakeTest is a test Teleop mode that is used to test the speed of the intake
 * Calibration can be changed live in FTC Dashboard
 */
@Config
@TeleOp (name = "Claw Test", group = "Testing")
public class ClawTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;

    private boolean left = false;
    private boolean right = false;

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    left = !left;
                }
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    right = !right;
                }

                if (left) {
                    robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_CLOSE);
                }else {
                    robot.clawLeftServo.setPosition(RobotSettings.CLAW_LEFT_OPEN);
                }

                if (right) {
                    robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_CLOSE);
                }else {
                    robot.clawRightServo.setPosition(RobotSettings.CLAW_RIGHT_OPEN);
                }

                robot.update();

                telemetry.update();
            }
        }

    }

}