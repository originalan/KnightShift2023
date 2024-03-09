package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * ArmTestPIDF is a test Teleop mode that is used to tune the movement of the Arm with a PIDF controller
 * Calibration can be changed live in FTC Dashboard
 */
@Config
@TeleOp (name = "Arm Test (PIDF)", group = "Tuning")
public class ArmTestPIDF extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private PIDFControl pid;
    private boolean turnedOff = false;
    private boolean gainSchedule = false;
    private double maxOutputPower = 0;
    private double maxVelocity = 0;
    public static int targetPos = 100;

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);
        pid = new PIDFControl();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Change 'targetPos' variable in this opmode using FTC Dashboard");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    turnedOff = !turnedOff;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    gainSchedule = !gainSchedule;
                }

                robot.armSubsystem.armState = Arm.ArmState.GO_TO_POSITION;
                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                int armPos = BulkReading.pArmLeftMotor;

                double pidPower;
                if (gainSchedule) {
                    pidPower = pid.calculatePID(targetPos, armPos, false, true);
                }else {
                    pidPower = pid.calculatePID(targetPos, armPos, false);
                }
//                double ffPower = pid.feedForwardCalculate(targetPos);
                double ffPower = pid.calculateFeedforward(targetPos, true);

                double power = pidPower + ffPower;

                if (power > PIDFControl.maxPower) {
                    power = PIDFControl.maxPower;
                }

                if (power > maxOutputPower) {
                    maxOutputPower = power;
                }

                if (turnedOff) {
                    robot.armSubsystem.targetPower = 0;
                }else {
                    robot.armSubsystem.targetPower = power;
                }

                robot.update();

                if (Math.abs(BulkReading.vArmLeftMotor) > maxVelocity) {
                    maxVelocity = BulkReading.vArmLeftMotor;
                }

                telemetry.addData("PID IS GAIN SCHEDULING?", gainSchedule);
                telemetry.addData("Target Position", targetPos);
                telemetry.addData("Arm actual position", armPos);
                telemetry.addData("Arm actual velocity", BulkReading.vArmLeftMotor);
                telemetry.addData("Arm calculated power", pidPower + ffPower);
                telemetry.addData("Arm initial encoder position", JVBoysSoccerRobot.initialArmPosition);
                telemetry.addData("Max power", maxOutputPower);
                telemetry.addData("Max velocity", maxVelocity);
//                telemetry.addData("kF right now", pid.KfCoefficients.get(targetPos));
                telemetry.update();
            }
        }

    }

}
