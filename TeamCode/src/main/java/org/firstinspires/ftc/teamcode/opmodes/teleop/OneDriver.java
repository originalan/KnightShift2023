package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PIDControl;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "OneDriver", group = "Testing")

public class OneDriver extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drivetrain;
    private PIDControl pid;


    @Override
    public void runOpMode() { // Remember that 'hardwareMap' is only visible in this method, not class

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        double reversed = 1.0;

        PIDControl pid = new PIDControl(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, pid);

        telemetry.addData("Status", "Initialized");
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
                double axial = -1 * gamepad1.left_stick_y * reversed; // pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x * reversed;
                double yaw = gamepad1.right_stick_x * reversed;

                // Moves drivetrain based on joystick values and updates telemetry with wheel powers
                drivetrain.goXYR(axial, lateral, yaw, telemetry);

                // PID control that adjusts for any irl inconsistencies with motor velocity
                // drivetrain.checkAndAdjustMotors();

                // Show elapsed game time and wheel power
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();

                if (currentGamepad1.a && !previousGamepad1.a) {
                    // changes back of robot to front (controls are based on front of robot)
                    reversed = (reversed == -1.0 ? 1.0 : -1.0);
                }
            }

        }

    }

}
