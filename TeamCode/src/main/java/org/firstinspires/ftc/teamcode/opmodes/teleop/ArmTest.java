package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDFControl;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;

/**
 * ArmTest is a test Teleop mode that is used to tune the movement of the Yellow Pixel Arm
 * Calibration can be changed live in FTC Dashboard
 * As of now, is outdated because the yellow pixel arm does not exist anymore
 */
@Config
@TeleOp (name = "Yellow Pixel Arm Test", group = "Testing")
public class ArmTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;
    public static int target = 0;

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        pid = new PIDFControl();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                robot.deliveryArm.slideState = DeliveryArm.ArmState.GO_TO_POSITION;
                int armPos = robot.linearSlideMotor.getCurrentPosition();
                double pidPower = pid.calculate(armPos, target, false);
                double ffPower = pid.feedForwardCalculate(target);

                robot.deliveryArm.targetPower = pidPower + ffPower;

                robot.update();

                telemetry.addData("Target", target);
                telemetry.addData("Arm actual position", armPos);
                telemetry.update();
            }
        }

    }

}
