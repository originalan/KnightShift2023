package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * "Inspired" from cruise control
 */
public abstract class Subsystem {

    public abstract void addTelemetry();

    public abstract void update();

    public abstract void stop();

}
