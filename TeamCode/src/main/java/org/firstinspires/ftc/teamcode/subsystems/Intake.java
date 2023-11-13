package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotSettings;

public class Intake extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;

    public enum IntakeState {
        OFF,
        ON,
    }

    public IntakeState intakeState = IntakeState.OFF;

    public Intake(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Intake");
        telemetry.addData("   Motor Power", robot.intakeMotor.getPower());
    }

    @Override
    public void update() {
        switch (intakeState) {
            case OFF:
                turnOff();
                break;
            case ON:
                moveForwards();
                // If intake is blocked, then current will spike and we will reverse motor to undo the blockage
                if(robot.intakeMotor.getCurrent(CurrentUnit.AMPS) > RobotSettings.INTAKE_CURRENT_THRESHOLD){
                    moveBackwards();
                }
                break;
        }
    }

    @Override
    public void stop() {
        turnOff();
    }

    public void moveForwards() {
        robot.intakeMotor.setPower(RobotSettings.INTAKE_DEFAULT_MOTOR_SPEED);
    }

    public void moveBackwards() {
        robot.intakeMotor.setPower(-1 * RobotSettings.INTAKE_DEFAULT_MOTOR_SPEED);
    }

    public void turnOff() {
        robot.intakeMotor.setPower(0);
    }

}
