package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDFControl;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;

/**
 * IntakeTest is a test Teleop mode that is used to test the speed of the intake
 * Calibration can be changed live in FTC Dashboard
 */
@Config
@TeleOp (name = "Intake Box Test", group = "Testing")
public class IntakeTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;

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

                if (Math.abs(currentGamepad1.left_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.ON;
                }
                else if (Math.abs(currentGamepad1.right_trigger) > 0.01) {
                    robot.intake.intakeState = Intake.IntakeState.REVERSE;
                }
                else {
                    robot.intake.intakeState = Intake.IntakeState.OFF;
                }

                robot.update();

                telemetry.update();
            }
        }

    }

}
