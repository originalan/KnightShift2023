package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.ArmSettings;
import org.firstinspires.ftc.teamcode.util.PIDFControl;

@TeleOp (name = "Full Arm Test", group = "Testing")
public class FullArmTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime1 = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;
    private boolean left = false, right = false;
    private boolean overrideLeft = false, overrideRight = false;
    private int overrideLeftCounter = 0, overrideRightCounter = 0;
    private double startingTimeLeft = 0, startingTimeRight = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Use bumpers to move claw pieces");
        telemetry.addLine("Use triggers to override set arm positions");
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

                if (left && right) {
                    robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                }
                if (left && !right) {
                    robot.clawSubsystem.clawState = Claw.ClawState.LEFT_OPEN;
                }
                if (right && !left) {
                    robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_OPEN;
                }
                if (!right && !left) {
                    robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
                }

                if (currentGamepad1.y && !previousGamepad1.y) {
                    robot.armSubsystem.encoderPosition = ArmSettings.position1;
                }
                if (currentGamepad1.a && !previousGamepad1.a) {
                    robot.armSubsystem.encoderPosition = ArmSettings.position2;
                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    robot.armSubsystem.encoderPosition = ArmSettings.position3;
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.encoderPosition = ArmSettings.positionBottom;
                }

                // Manual "override" of PIDF control of arm
                overrideLeft = currentGamepad1.left_trigger > 0.01;

                overrideRight = currentGamepad1.right_trigger > 0.01;

                if (overrideLeft && !overrideRight) {
                    overrideLeftCounter++;
                    if (overrideLeftCounter == 1) {
                        startingTimeLeft = runtime1.seconds();
                    }
                    double difference = runtime1.seconds() - startingTimeLeft;
                    if (difference > 0.05) { // 20 encoder ticks change per second
                        robot.armSubsystem.encoderPosition++;
                        runtime1.reset();
                    }
                }else {
                    overrideLeftCounter = 0;
                }


                if (overrideRight && !overrideLeft) {
                    overrideRightCounter++;
                    if (overrideRightCounter == 1) {
                        startingTimeRight = runtime2.seconds();
                    }
                    double difference2 = runtime2.seconds() - startingTimeRight;
                    if (difference2 > 0.05) { // 20 encoder ticks change per second
                        robot.armSubsystem.encoderPosition--;
                        runtime2.reset();
                    }
                }else {
                    overrideRightCounter = 0;
                }

                robot.armSubsystem.setArmEncoderPosition(robot.armSubsystem.encoderPosition);

                // ARM PIVOT SERVO
                // NEED TO WRITE CODE
                // 1.0 is 180 degrees
                // 1120 is for 360 degrees
                // 1120 / 2 is for 180 degrees
                // 1120/2 / 180.0 = x / 1.0
                // solve for x, x + initial servo pos = arm pivot servo pos

                robot.armSubsystem.addTelemetry();
                robot.clawSubsystem.addTelemetry();
                telemetry.update();
                robot.update();
            }
        }

    }
}
