package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.purepursuit.RobotMovement;

public class PurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() {


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                RobotMovement.goToPosition(358 / 2, 358 / 2, 0.3, Math.toRadians(90), 0.3); // should change 90 to 0

            }
        }

    }

}
