package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

@Disabled
@TeleOp(name = "Reset Everything", group = "Testing")
public class ResetEverything extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;

    public void runOpMode() {

        telemetry.addLine("WAIT FOR INITIALIZATION MESSAGE BEFORE PRESSING START");
        telemetry.update();

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

                robot.launcherSubsystem.launcherState = AirplaneLauncher.LauncherState.SETUP;

                // CAN SET MOTOR STRING WITH THIS CODE:
                if (currentGamepad1.dpad_right) {
                    if (currentGamepad1.left_bumper) {
                        robot.rigRightMotor.setPower(-1 * RobotSettings.RIGGING_RESET_MOTOR_SPEED);
                    }
                    if (currentGamepad1.right_bumper) {
                        robot.rigRightMotor.setPower(RobotSettings.RIGGING_RESET_MOTOR_SPEED);
                    }
                }
                if (currentGamepad1.dpad_left) {
                    if (currentGamepad1.left_bumper) {
                        robot.rigLeftMotor.setPower(RobotSettings.RIGGING_RESET_MOTOR_SPEED);
                    }
                    if (currentGamepad1.right_bumper) {
                        robot.rigLeftMotor.setPower(-1 * RobotSettings.RIGGING_RESET_MOTOR_SPEED);
                    }
                }

                robot.update();
                telemetry.update();

            }
        }

    }

}
