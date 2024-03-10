package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.settings.RobotSettings;

/**
 * AirplaneLauncherTest is a test Teleop mode that is used purely to test the airplane launcher.
 */
@TeleOp (name = "Rigging Test", group = "Testing")
public class RiggingTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private double rigWaitTime = 0;

    private enum RiggingControlsState {
        DOWN_WAIT,
        DOWN,
        UP,
        HANGING,
        NOTHING
    }
    private RiggingControlsState hangState = RiggingControlsState.DOWN;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Press X to bring arms up, x again to bring them down");
        telemetry.addLine("No motor code yet");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

//                if (currentGamepad1.x && !previousGamepad1.x) {
//                    isRigging = !isRigging;
//                }
//
//                if (isRigging) {
//                    robot.riggingSubsystem.hang();
//                }else {
//                    robot.riggingSubsystem.noHang();
//                }

                switch (hangState) {
                    case UP:
                        robot.riggingSubsystem.riggingState = Rigging.RiggingState.RIG;
                        if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                            hangState = RiggingControlsState.HANGING;
                        }
                        if (currentGamepad1.x && !previousGamepad1.x) {
                            rigWaitTime = runtime.seconds();
                            robot.rigRightServo.getController().pwmEnable();
                            robot.rigLeftServo.getController().pwmEnable();
                            hangState = RiggingControlsState.DOWN_WAIT;
                        }
                        break;
                    case DOWN:
                        robot.riggingSubsystem.riggingState = Rigging.RiggingState.NOTHING;
                        robot.rigRightServo.getController().pwmDisable();
                        robot.rigLeftServo.getController().pwmDisable();
                        if (currentGamepad1.x && !previousGamepad1.x) {
                            robot.rigRightServo.getController().pwmEnable();
                            robot.rigLeftServo.getController().pwmEnable();
                            hangState = RiggingControlsState.UP;
                        }
                        break;
                    case DOWN_WAIT:
                        robot.riggingSubsystem.riggingState = Rigging.RiggingState.NO_RIG;
                        if (runtime.seconds() - rigWaitTime > 1.0) {
                            hangState = RiggingControlsState.DOWN;
                        }
                        break;
                    case HANGING:
                        robot.riggingSubsystem.riggingState = Rigging.RiggingState.NOTHING;
                        robot.rigRightServo.getController().pwmDisable();
                        robot.rigLeftServo.getController().pwmDisable();
                        if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                            robot.rigRightMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                            robot.rigLeftMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
                        }else {
                            robot.rigRightMotor.setPower(0);
                            robot.rigLeftMotor.setPower(0);
                        }
                        if (currentGamepad1.x && !previousGamepad1.x) {
                            rigWaitTime = runtime.seconds();
                            robot.rigRightServo.getController().pwmEnable();
                            robot.rigLeftServo.getController().pwmEnable();
                            hangState = RiggingControlsState.DOWN_WAIT;
                        }
                        break;
                    case NOTHING:
                        break;
                }

                if (currentGamepad1.dpad_left) {
                    robot.rigLeftMotor.setPower(-1.0 * RobotSettings.RIGGING_MOTOR_SPEED * 0.5);
                }else {
                    robot.rigLeftMotor.setPower(0);
                }

                if (currentGamepad1.dpad_right) {
                    robot.rigRightMotor.setPower(-1.0 * RobotSettings.RIGGING_MOTOR_SPEED * 0.5);
                }else {
                    robot.rigRightMotor.setPower(0);
                }

                robot.riggingSubsystem.addTelemetry();
                robot.riggingSubsystem.update();
                telemetry.update();

            }
        }

    }

}
