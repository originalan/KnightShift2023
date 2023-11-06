package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class Auto1 extends LinearOpMode {

    private Drivetrain drivetrain;
    @Override
    public void runOpMode() {

        drivetrain = new Drivetrain(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

            }

        }

    }

}
