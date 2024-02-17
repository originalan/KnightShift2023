package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.purepursuit.RobotMovement.followCurve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.purepursuit.RobotMovement;

import java.util.ArrayList;

@Disabled
@TeleOp (name = "Pure Pursuit", group = "Testing")
public class PurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("is not functional yet, many bugs");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                ArrayList<CurvePoint> allPoints = new ArrayList<>();
                allPoints.add(new CurvePoint(0, 0, 1, 1, 50, Math.toRadians(50), 1.0));

                followCurve(allPoints, Math.toRadians(90));
            }
        }

    }

}
