package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.settings.ArmSettings;
import org.firstinspires.ftc.teamcode.util.BulkReading;
import org.firstinspires.ftc.teamcode.util.PIDFControl;

@TeleOp (name = "Arm Pivot Test", group = "Testing")
public class ArmPivotTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private BulkReading bulkReading;
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
        bulkReading = new BulkReading(robot, telemetry, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.addLine("Use dpad down to move pivot claw (left)");
        telemetry.addLine("Press X to move the actual arm to 'targetPosition'");
        telemetry.addLine("Press left or right bumper to release/activate servo tension");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                bulkReading.readAll();

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                    move = !move;
                }

                if (move) {
                    robot.clawPivotRightServo.setPosition(ArmSettings.ARM_PIVOT_TEST_POS);
                }else {
                    robot.clawPivotRightServo.setPosition(ArmSettings.ARM_PIVOT_SERVO_REST);
                }

                robot.update();

                robot.clawSubsystem.addTelemetry();
                robot.armSubsystem.addTelemetry();
                telemetry.update();

            }
        }

    }
}
