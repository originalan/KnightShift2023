package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DoesNothing", group = "Autonomous Opmode 11.19")
public class DoesNothing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // in case we are so bad that we can't do anything to help during auto

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

            }
        }

    }
}
