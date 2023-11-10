package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDControl;

public class AirplaneLauncher extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private PIDControl pid;

    private Servo servo;

    public enum LauncherState {
        OFF,
        ZONE_ONE_OR_BUST
    }
    public LauncherState launcherState = LauncherState.OFF;

    public AirplaneLauncher(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        servo = hwMap.servo.get("AirplaneLauncher");
        servo.setPosition(0);

    }

    @Override
    public void addTelemetry() {

        telemetry.addData("Airplane Launcher Servo Current Position", servo.getPosition());

    }

    @Override
    public void update() {

        switch (launcherState) {

            case OFF:
                notYet();
                break;
            case ZONE_ONE_OR_BUST:
                ZONE_ONE_OR_BUST();
                break;

        }

    }

    public void ZONE_ONE_OR_BUST() {

        servo.setPosition(1.0);

    }

    public void notYet() {

        servo.setPosition(0);

    }

}
