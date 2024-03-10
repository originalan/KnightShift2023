package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.util.SuperController;

/**
 * ArmTestPIDF is a test Teleop mode that is used to tune the movement of the Arm with a PIDF controller
 * Calibration can be changed live in FTC Dashboard
 */
@Config
@TeleOp (name = "Arm Test (everything but MP)", group = "Tuning")
public class ArmTestPIDF extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private SuperController superController;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;

    private boolean left = true, right = true;

    private boolean turnedOff = false;
    public static boolean feedforward_g = true, pid = false;
    public static boolean feedforward_kvka = false, fullstate = false;
    public static int targetPos = 100;
    public static double targetVelocity = 0; // enocder ticks per second
    public static double targetAcceleration = 0;

    private enum ArmTestState {
        CLOSED,
        INTAKING,
        DROP_POS,
        NOTHING,
        RESET
    }
    private ArmTestState armTestState = ArmTestState.CLOSED;

    @Override
    public void runOpMode() {

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);
        superController = new SuperController();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Change 'targetPos' variable in this opmode using FTC Dashboard");
        telemetry.addLine("Same controls as TwoDriver for arm");
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

                armControls();

                robot.update();

                telemetry.update();
            }
        }

    }

    private void armControls() {
        switch (armTestState) {
            case CLOSED:
                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                robot.armSubsystem.pivotState = Arm.PivotState.REST;

                // reset button
                if (currentGamepad1.right_bumper && currentGamepad1.left_bumper) {
                    armTestState = ArmTestState.RESET;
                }
//                if (currentGamepad2.right_bumper && currentGamepad2.left_bumper && !previousGamepad2.right_bumper) {
//                    intakeState = IntakeControlsState.RESET;
//                }

                if (currentGamepad1.x && !previousGamepad1.x) {
                    armTestState = ArmTestState.DROP_POS;
                }

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up /*  && withinRange(BulkReading.pArmLeftMotor, -5, 5) */ ) {
                    left = false; // left claw open
                    right = false; // right claw open
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.INTAKING;
                }
                break;
            case INTAKING:
                robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                clawSidePieceControls(true);

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.CLOSED;
                }
                break;
            case DROP_POS:
                robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
                robot.armSubsystem.armState = Arm.ArmState.PIDF_TEST;
                clawSidePieceControls(false);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    armTestState = ArmTestState.CLOSED;
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                }
                break;
            case RESET:
                robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                armTestState = ArmTestState.CLOSED;
                break;
            case NOTHING:
                break;
        }
    }

    private void clawSidePieceControls(boolean reversed) {
        if (reversed) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                right = !right;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                left = !left;
            }
        }else {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                left = !left;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                right = !right;
            }
        }

        if (left && right) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
        }
        if (left && !right) {
            robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
        }
        if (right && !left) {
            robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
        }
        if (!right && !left) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
        }
    }

}
