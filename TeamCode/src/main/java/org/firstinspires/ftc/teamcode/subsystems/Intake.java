package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    // NEED TO CHANGE THIS VALUE BASED ON EXPERIMENTS
    private final double currentThreshold = 10000;

    public enum IntakeState {
        OFF,
        FORWARDS,
        BACKWARDS
    }

    public IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addData("Intake Power", robot.intakeMotor.getPower());
    }

    @Override
    public void update() {
        switch (intakeState) {
            case OFF:
                turnOff();
                break;
            case FORWARDS:
                moveForwards();
                break;
            case BACKWARDS:
                moveBackwards();

                // If intake is blocked, then current will spike and we will reverse motor to undo the blockage
                if(robot.intakeMotor.getCurrent(CurrentUnit.AMPS) > currentThreshold){
                    moveForwards();
                }
                break;
        }
    }

    @Override
    public void stop() {
        turnOff();
    }

    public void moveForwards() {
        robot.intakeMotor.setPower(1);
    }

    public void moveBackwards() {
        robot.intakeMotor.setPower(-1);
    }

    public void turnOff() {
        robot.intakeMotor.setPower(0);
    }

}
