package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.SuperController;

@Disabled
@Config
@TeleOp (name = "Arm Test Back and Forth", group = "Tuning")
public class ArmTestBackForth extends LinearOpMode {

    private JVBoysSoccerRobot robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime backForthTimer = new ElapsedTime();
    private BulkReading bulkReading;
    private SuperController superController;

    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private boolean turnedOff = true;
    private boolean left = true, right = true;
    public static int targetPos = 500;

    private enum ArmTestState {
        NOTHING,
        RESET,
        IN_POSITION,
        BACK_AND_FORTH
    }

    private enum BackForthState {
        WAIT,
        BACK,
        WAIT2,
        FORTH
    }
    private ArmTestState armTestState = ArmTestState.NOTHING;
    private BackForthState backForthState = BackForthState.FORTH;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Back and forth arm movement opmode for tuning");
        telemetry.addLine("Press X, then dpad down to activate, then x again to turn off");
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                armControls();

                robot.update();

                telemetry.addData("Target Position", targetPos);
                telemetry.addData("Actual Position", BulkReading.pArmLeftMotor);
                telemetry.addLine("===========================");

                telemetry.addData("Target Position (rn)", robot.armSubsystem.getMP().getInstantPosition());
                telemetry.addData("Target Velocity (rn)", "%.4f", robot.armSubsystem.getMP().getInstantVelocity());

                telemetry.addData("Actual Velocity (reading)", "%.4f", BulkReading.vArmLeftMotor);

                telemetry.update();

            }
        }
    }

    public void armControls() {
        switch (armTestState) {
            case NOTHING:
                robot.armSubsystem.armState = Arm.ArmState.NOTHING;

                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                    robot.armSubsystem.setMotionProfile(120);
                    armTestState = ArmTestState.IN_POSITION;
                }

                if (currentGamepad1.left_bumper && currentGamepad1.right_bumper) {
                    armTestState = ArmTestState.RESET;
                }
                break;
            case RESET:
                robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                armTestState = ArmTestState.NOTHING;
                break;
            case BACK_AND_FORTH:
                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.armState = Arm.ArmState.NOTHING;
                    armTestState = ArmTestState.NOTHING;
                }

                // nested state machine because i'm actually mentally unwell
                double time = backForthTimer.seconds();
                switch (backForthState) {
                    case FORTH:
                        if (!robot.armSubsystem.getMP().isBusy()) {
                            backForthTimer.reset();
                            backForthState = BackForthState.WAIT;
                        }
                        break;
                    case WAIT:
                        if (time > 3.0) {
                            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                            robot.armSubsystem.setMotionProfile(630);
                            backForthState = BackForthState.BACK;
                        }
                        break;
                    case BACK:
                        if (!robot.armSubsystem.getMP().isBusy()) {
                            backForthTimer.reset();
                            backForthState = BackForthState.WAIT2;
                        }
                        break;
                    case WAIT2:
                        if (time > 3.0) {
                            robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE;
                            robot.armSubsystem.setMotionProfile(120);
                            backForthState = BackForthState.FORTH;
                        }
                        break;
                }
                break;
            case IN_POSITION:
                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    armTestState = ArmTestState.BACK_AND_FORTH;
                    backForthState = BackForthState.FORTH;
                    backForthTimer.reset();
                }
                break;
        }
    }
}
