package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.ArmSettings;
import org.firstinspires.ftc.teamcode.util.PIDFControl;

@TeleOp (name = "Arm Pivot Test", group = "Testing")
public class ArmPivotTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;
    private boolean move = false;
    public static int targetPosition = 100;
    private boolean turnedOff = true;
    private boolean servosOff = false;
    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        pid = new PIDFControl();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Use dpad down to move pivot claw (left)");
        telemetry.addLine("Press X to move the actual arm to 'targetPosition'");
        telemetry.addLine("Press left or right bumper to release/activate servo tension");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    move = !move;
                }

                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    servosOff = !servosOff;
                }
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    servosOff = !servosOff;
                }

                if (servosOff) {
                    robot.clawPivotRightServo.getController().pwmDisable();
                    robot.clawPivotLeftServo.getController().pwmDisable();
                }else {
                    robot.clawPivotRightServo.getController().pwmEnable();
                    robot.clawPivotLeftServo.getController().pwmEnable();
                }

                if (move) {
                    robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_TEST_POS);
                }else {
                    robot.clawPivotLeftServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_REST);
                }

                if (currentGamepad1.x && !previousGamepad1.x) {
                    turnedOff = !turnedOff;
                }

                robot.armSubsystem.armState = Arm.ArmState.GO_TO_POSITION;
                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                int armPos = robot.armLeftMotor.getCurrentPosition();

                double pidPower = pid.calculate(targetPosition, armPos, false);
                double ffPower = pid.feedForwardCalculate(armPos);

                double power = pidPower + ffPower;

                if (turnedOff) {
                    robot.armSubsystem.targetPower = 0;
                }else {
                    robot.armSubsystem.targetPower = power;
                }

                robot.update();

                robot.clawSubsystem.addTelemetry();
                robot.armSubsystem.addTelemetry();
                telemetry.update();

            }
        }

    }
}
