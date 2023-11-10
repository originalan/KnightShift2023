package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PIDControl;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;

@TeleOp(name = "OneDriver", group = "Testing")

public class OneDriver extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drivetrain;
    private Rigging rig;
    private PIDControl pid;
    private Intake intake;

    private boolean intakeOn = false;
    private boolean servoMove = false;


    @Override
    public void runOpMode() { // Remember that 'hardwareMap' is only visible in this method, not class

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        double reversed = 1.0;

        PIDControl pid = new PIDControl(hardwareMap, telemetry);
        drivetrain = new Drivetrain(hardwareMap, pid, telemetry);
        rig = new Rigging(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        // WAIT FOR THIS TELEMETRY MESSAGE BEFORE PRESSING START because IMU takes a while to be initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Elapsed time", runtime.toString());
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            // Put run blocks here.

            // Run until end of match (driver presses STOP)
            while (opModeIsActive()) {
                // Put loop blocks here.

                // Keep at top of loop, is used to check if buttons are pressed
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Records joystick values
//                double axial = -1 * gamepad1.left_stick_y * reversed; // pushing stick forward gives negative value
//                double lateral = gamepad1.left_stick_x * reversed;
//                double yaw = gamepad1.right_stick_x * reversed;

                // You have to take values as if the robot is 90 degrees rotated because the expansion hub is placed 90 degrees rotated
                // I rotated (x,y) 90 degrees CW to (y, -x)
                double axialIMU = -1 * gamepad1.left_stick_x * reversed;
                double lateralIMU = -1 * gamepad1.left_stick_y * reversed;
                double yawIMU = gamepad1.right_stick_x * reversed;

                // Prem said this is good cuz you can easily see what is wrong if robot strafes off
                double axialIMU2 = gamepad1.left_stick_x * reversed;
                double lateralIMU2 = -1 * gamepad1.right_stick_y * reversed;
                double yawIMU2 = (gamepad1.right_trigger - gamepad1.left_trigger) * reversed;

                // Moves drivetrain based on joystick values and updates telemetry with wheel powers
                // drivetrain.goXYR(axial, lateral, yaw, telemetry);

                // Moves drivetrain on a field orientated drive and updates telemetry with wheel powers
                drivetrain.goXYRIMU(axialIMU2, lateralIMU2, yawIMU2);

                // PID control that adjusts for any irl inconsistencies with motor velocity
                // drivetrain.checkAndAdjustMotors();

                // Show elapsed game time and wheel power
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                rig.addTelemetry();
                intake.addTelemetry();
                drivetrain.addTelemetry();

//                if (currentGamepad1.a && !previousGamepad1.a) {
//                    // changes back of robot to front (controls are based on front of robot)
//                    reversed = (reversed == -1.0 ? 1.0 : -1.0);
//                }
                if (currentGamepad1.b && !previousGamepad1.b) {
                    drivetrain.resetInitYaw();
                }
                if (currentGamepad1.x && !previousGamepad1.x) {
                    servoMove = !servoMove;
                }
                if (currentGamepad1.y && !previousGamepad1.y) {
                    intakeOn = !intakeOn;
                }

                if (servoMove) {
                    rig.hang();
                }else {
                    rig.undoHang();
                }

                if (intakeOn) {
                    intake.moveBackwards();
                }else {
                    intake.turnOff();
                }

                telemetry.update();
            }

        }

    }

}
