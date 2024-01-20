package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.FullStateFeedback;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;

/**
 * ArmTestFullState is a test Teleop mode that is used to tune the movement of the Arm with a Full State Feedback controller
 * Calibration can be changed live in FTC Dashboard
 */
@Config
@TeleOp (name = "Arm Test (Full State Feedback)", group = "Testing")
public class ArmTestFullState extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;
    private FullStateFeedback controller;
    private boolean armOn = false;
    public static int targetPos = 0;

    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new JVBoysSoccerRobot(hardwareMap, telemetry);
        pid = new PIDFControl();
        controller = new FullStateFeedback(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

//                robot.deliveryArm.armState = DeliveryArm.ArmState.GO_TO_POSITION;
                int armPos = robot.deliveryArmMotor.getCurrentPosition();

//                double pidPower = pid.calculate(armPos, targetPos, false);
//                double ffPower = pid.feedForwardCalculate(targetPos);
//
//                robot.deliveryArm.targetPower = pidPower + ffPower;

//                double power = controller.calculate(targetPos, 0, armPos, robot.deliveryArmMotor.getVelocity());
//                robot.deliveryArm.targetPower = power;
                if (currentGamepad1.x && !previousGamepad1.x) {
                    armOn = !armOn;
                }

                if (armOn) {
                    robot.deliveryArm.armState = DeliveryArm.ArmState.TEST;
                    telemetry.addLine("ARM IS ON");
                }else {
                    robot.deliveryArm.armState = DeliveryArm.ArmState.AT_REST;
                    telemetry.addLine("ARM IS OFF");
                }

                robot.update();

                telemetry.addData("Target Position", targetPos);
                telemetry.addData("Arm actual position", armPos);
                telemetry.addData("Arm actual velocity", robot.deliveryArmMotor.getVelocity());
                telemetry.addData("Arm initial encoder position", JVBoysSoccerRobot.initialArmPosition);
                telemetry.update();
            }
        }

    }

}