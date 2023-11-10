package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

/**
 * Robot super system - JV BOYS SOCCER TEAM
 */
public class JVBoysSoccerRobot {

    private HardwareMap hwMap;
    private Telemetry telemetry;

    public Drivetrain drivetrain;
    public Intake intake;
    public LinearSlide slide;
    public Rigging rig;
    public AirplaneLauncher launcher;

    private List<Subsystem> subsystems;

    public JVBoysSoccerRobot(HardwareMap hwMap, Telemetry telemetry) {

        this.hwMap = hwMap;
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hwMap, telemetry);
        intake = new Intake(hwMap, telemetry);
        slide = new LinearSlide(hwMap, telemetry);
        rig = new Rigging(hwMap, telemetry);
        launcher = new AirplaneLauncher(hwMap, telemetry);

        subsystems = Arrays.asList(drivetrain, intake, slide, rig, launcher);

    }

    public void addTelemetry() {

        for (Subsystem s : subsystems) {

            s.addTelemetry();

        }

    }

}
