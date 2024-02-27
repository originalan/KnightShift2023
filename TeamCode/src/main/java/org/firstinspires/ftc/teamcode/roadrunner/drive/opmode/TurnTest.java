package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while (opModeInInit()) {
            drive.raiseArm(); // added for our specific robot
        }

        waitForStart();

        if (isStopRequested()) return;

        drive.turnAsync(Math.toRadians(ANGLE));
        while (opModeIsActive() && !isStopRequested()) {
            drive.raiseArm(); // added for our specific robot
            drive.update();
        }
    }
}
