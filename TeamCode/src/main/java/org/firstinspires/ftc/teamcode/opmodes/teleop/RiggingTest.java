package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * AirplaneLauncherTest is a test Teleop mode that is used purely to test the airplane launcher.
 */
@TeleOp (name = "Rigging Test", group = "Testing")
public class RiggingTest extends LinearOpMode {

    private boolean isRigging = false;
    private boolean rigStringMove = false;

    @Override
    public void runOpMode() throws InterruptedException {

        JVBoysSoccerRobot robot = new JVBoysSoccerRobot(hardwareMap, telemetry);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();



        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    isRigging = !isRigging;
                }

                if (isRigging) {
                    robot.riggingSubsystem.hang();
                }else {
                    robot.riggingSubsystem.noHang();
                }
//
//                if (!rigStringMove) {
//                    if (isRigging) {
//                        robot.rigRightServo.getController().pwmEnable();
//                        robot.rigLeftServo.getController().pwmEnable();
//                        robot.riggingSubsystem.hang();
//                    }else {
//                        robot.rigRightServo.getController().pwmDisable();
//                        robot.rigLeftServo.getController().pwmDisable();
//                    }
//                }else {
//                    robot.rigRightServo.getController().pwmDisable();
//                    robot.rigLeftServo.getController().pwmDisable();
//                }
//
//                if (isRigging && (currentGamepad1.dpad_right || currentGamepad1.dpad_left)) {
//                    rigStringMove = true;
//                    robot.rigRightMotor.setPower(-1 * RobotSettings.RIGGING_MOTOR_SPEED);
//                    robot.rigLeftMotor.setPower(RobotSettings.RIGGING_MOTOR_SPEED);
//                }
//                else {
//                    robot.rigRightMotor.setPower(0);
//                    robot.rigLeftMotor.setPower(0);
//                }

                robot.riggingSubsystem.addTelemetry();
                robot.riggingSubsystem.update();
                telemetry.update();

            }
        }

    }

}
