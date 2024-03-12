package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;

@Config
@TeleOp (name = "Arm Test (Motion Profiling)", group = "Tuning")
public class ArmTestMP extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double delayTime = 0;
    private double doubleCheckWaitTime = 0;
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private boolean leftClosed = true, rightClosed = true;
    public static int targetPos = 500;

    private enum ArmTestState {
        CLOSED,
        DELAY,
        GO_BACK_DOWN,
        DOUBLE_CHECK,
        INTAKING,
        INTAKING_AUTO,
        DROP_POS,
        RESET,
        NOTHING
    }

    private ArmTestState armTestState = ArmTestState.CLOSED;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Change 'targetPos' variable in this opmode using FTC Dashboard");
        telemetry.addLine("Same controls as TwoDriver for arm");
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                armControls2();
//                armControls();

                robot.update();

                telemetry.addData("Target/Goal Position", targetPos);
                telemetry.addData("Actual Position", BulkReading.pArmLeftMotor);
                telemetry.addLine("===========================");

                telemetry.addData("Target Position (rn)", robot.armSubsystem.getMP().getInstantPosition());
                telemetry.addData("Target Velocity (rn)", "%.4f", robot.armSubsystem.getMP().getInstantVelocity());

                telemetry.addData("Actual Velocity (reading)", "%.4f", BulkReading.vArmLeftMotor);

                telemetry.update();
            }
        }

    }

    private void armControls2() {
        switch (armTestState) {
            case CLOSED:
//                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
//                robot.armSubsystem.pivotState = Arm.PivotState.REST;

                // reset button
                if (currentGamepad1.right_bumper && currentGamepad1.left_bumper) {
                    armTestState = ArmTestState.RESET;
                }

                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfileTest(targetPos);
                    armTestState = ArmTestState.DROP_POS;
                }

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up /*  && withinRange(BulkReading.pArmLeftMotor, -5, 5) */ ) {
                    leftClosed = false; // left claw open
                    rightClosed = false; // right claw open
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.INTAKING;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down /*  && withinRange(BulkReading.pArmLeftMotor, -5, 5) */ ) {
                    leftClosed = false; // left claw open
                    rightClosed = false; // right claw open
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.INTAKING_AUTO;
                }
                break;
            case DELAY:
                // robot arm is moving back down to intaking / closed position
                // only after 0.5 seconds from button press does claw servos move (so they don't hit the backdrop)
                if (runtime.seconds() - delayTime > 0.25) {
                    robot.armSubsystem.pivotState = Arm.PivotState.REST;
                    robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;

                    armTestState = ArmTestState.GO_BACK_DOWN;
                }
                break;
            case GO_BACK_DOWN:
                // robot arm is moving back down to intaking / closed position
                if (!robot.armSubsystem.getMP().isBusy()) {
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;

                    doubleCheckWaitTime = runtime.seconds();
                    armTestState = ArmTestState.DOUBLE_CHECK;
                }
                break;
            case DOUBLE_CHECK:
                // let arm settle for 0.5 seconds
                if (runtime.seconds() - doubleCheckWaitTime > 0.5) {
                    armTestState = ArmTestState.RESET;
                }
                break;
            case INTAKING:
                robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                clawSidePieceControls(true);

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    armTestState = ArmTestState.RESET;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.INTAKING_AUTO;
                }
                break;
            case INTAKING_AUTO:
                robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                robot.clawSubsystem.clawState = Claw.ClawState.AUTO_DETECT;

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    armTestState = ArmTestState.INTAKING;
                }
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    armTestState = ArmTestState.RESET;
                }
                break;
            case DROP_POS:
                robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
                clawSidePieceControls(false);

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfileTest(ArmSettings.positionBottom);
                    delayTime = runtime.seconds();
                    armTestState = ArmTestState.DELAY;
                }
                break;
            case RESET:
                robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.armSubsystem.pivotState = Arm.PivotState.REST;
                robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
                robot.armSubsystem.armState = Arm.ArmState.NOTHING;

                armTestState = ArmTestState.CLOSED;
                break;
            case NOTHING:
                break;
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
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE_TEST;
                    robot.armSubsystem.setMotionProfileTest(targetPos);
                    armTestState = ArmTestState.DROP_POS;
                }

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up && !robot.armSubsystem.getMP().isBusy() ) {
                    leftClosed = false; // left claw open
                    rightClosed = false; // right claw open
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.INTAKING;
                }
                break;
            case GO_BACK_DOWN:
                if (!robot.armSubsystem.getMP().isBusy() && withinRange(BulkReading.pArmLeftMotor, -10, 10)) {
                    armTestState = ArmTestState.RESET;
                }
                break;
            case INTAKING:
                robot.armSubsystem.pivotState = Arm.PivotState.GROUND;
                clawSidePieceControls(true);

                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.RESET;
                }
                break;
            case DROP_POS:
                robot.armSubsystem.pivotState = Arm.PivotState.AUTO_CALIBRATE;
                clawSidePieceControls(false);

                if (currentGamepad1.x && !previousGamepad1.x && !robot.armSubsystem.getMP().isBusy()) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE_TEST;
                    robot.armSubsystem.setMotionProfileTest(ArmSettings.positionBottom);
                    armTestState = ArmTestState.GO_BACK_DOWN;
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
                rightClosed = !rightClosed;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                leftClosed = !leftClosed;
            }
        }else {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                leftClosed = !leftClosed;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                rightClosed = !rightClosed;
            }
        }

        if (leftClosed && rightClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_CLOSED;
        }
        if (leftClosed && !rightClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.LEFT_CLAW_OPEN;
        }
        if (rightClosed && !leftClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.RIGHT_CLAW_OPEN;
        }
        if (!rightClosed && !leftClosed) {
            robot.clawSubsystem.clawState = Claw.ClawState.BOTH_OPEN;
        }
    }

    public boolean withinRange(double value, double bottom, double top) {

        return value >= bottom && value <= top;

    }

}
