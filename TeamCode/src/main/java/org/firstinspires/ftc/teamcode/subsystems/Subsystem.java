package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Subsystem is an abstract class used to easily integrate different hardware modules into the super robot class.
 */
public abstract class Subsystem {

    public abstract void addTelemetry();

    public abstract void update();

    public abstract void stop();

}
