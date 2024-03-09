package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Config
@TeleOp (name = "Arm Test (Motion Profiling)", group = "Tuning")
public class ArmTestMP extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
    private MotionProfile mp;
    private Gamepad currentGamepad1;
    private Gamepad previousGamepad1;
    private boolean turnedOff = true;
    private boolean left = true, right = true;
    public static int targetPos = 500;

    private enum ArmTestState {
        CLOSED,
        INTAKING,
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
        mp = new MotionProfile();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Change 'targetPos' variable in this opmode using FTC Dashboard");
        telemetry.addLine("Use dpad down to set a new targetPos after changing it in dashboard");
        telemetry.addLine("Use dpad up to change gain scheduling");
        telemetry.update();

        double pos1 = 0;
        double pos2 = 0;
        double time1 = 0;
        double time2 = 0;

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

                pos2 = pos1;
                pos1 = BulkReading.pArmLeftMotor;
                time2 = time1;
                time1 = runtime.seconds();

                telemetry.addData("Target Position (rn)", mp.getInstantPosition());
                telemetry.addData("Target Velocity (rn)", "%.4f", mp.getInstantVelocity());

                telemetry.addData("Actual Velocity (reading)", "%.4f", BulkReading.vArmLeftMotor);
                telemetry.addData("Actual Velocity (calculation)", "%.4f", (pos2 - pos1) / (time2 - time1));

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
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE_TEST;
                    robot.armSubsystem.setMotionProfileTest(targetPos);
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
                clawSidePieceControls(false);

                if (currentGamepad1.x && !previousGamepad1.x) {
                    robot.armSubsystem.armState = Arm.ArmState.MOTION_PROFILE_TEST;
                    robot.armSubsystem.setMotionProfileTest(targetPos);
                    armTestState = ArmTestState.CLOSED;
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
