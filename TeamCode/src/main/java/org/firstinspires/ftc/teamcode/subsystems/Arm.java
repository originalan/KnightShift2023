package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * DeliveryArm is a Subsystem representing all delivery arm hardware movement
 */
@Config
public class Arm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;

    public double targetPower = 0; // used purely for GO_TO_POSITION arm state, PIDF test
    public int encoderPosition = 0;
    public double projectedPower = 0;
    public boolean overridePowerForward = false;
    public boolean overridePowerBackward = false;

    public static int positionBottom = 0;
    public static int position1 = 50;
    public static int position2 = 100;
    public static int position3 = 150;

    public enum ArmState {
        AT_REST, // no power
        BOTTOM, // intaking position
        POS1, // Very top position of arm
        POS2, // Slightly lower
        POS3, // Lowest, good for mosaics at the beginning of teleop
        GO_TO_POSITION,
        NOTHING
    }

    public ArmState armState = ArmState.NOTHING;

    public Arm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.pid = new PIDFControl();

        JVBoysSoccerRobot.initialArmPosition = robot.armLeftMotor.getCurrentPosition();
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Delivery Arm");
        telemetry.addData("   Motor Position Encoder Value", "%d", robot.armLeftMotor.getCurrentPosition());
    }

    @Override
    public void update() {
        switch (armState) {
            case GO_TO_POSITION:
                noEncoders();
                setArmPower(targetPower, overridePowerForward, overridePowerBackward);
                break;
            case BOTTOM:
                noEncoders();
                projectedPower = pid.calculate(JVBoysSoccerRobot.initialArmPosition + 3,
                        robot.armLeftMotor.getCurrentPosition(),
                        false);
                setArmPower(projectedPower, overridePowerForward, overridePowerBackward);
                break;
            case POS1:
                noEncoders();
                encoderPosition = 0;
                setArmPower(encoderPosition);
                break;
            case POS2:
                noEncoders();
                break;
            case POS3:
                noEncoders();
                break;
            case AT_REST:
                noEncoders();
                projectedPower = 0;
                setArmPower(projectedPower, overridePowerForward, overridePowerBackward);
                break;
            case NOTHING:
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void setArmPower(double power, boolean overrideForward, boolean overrideBackward) {
        if (overrideForward) {
            robot.armLeftMotor.setPower(power - RobotSettings.ARM_OVERRIDE_POWER);
            robot.armRightMotor.setPower(power - RobotSettings.ARM_OVERRIDE_POWER);
        }else if (overrideBackward) {
            robot.armLeftMotor.setPower(power + RobotSettings.ARM_OVERRIDE_POWER);
            robot.armRightMotor.setPower(power + RobotSettings.ARM_OVERRIDE_POWER);
        }else {
            robot.armLeftMotor.setPower(power);
            robot.armRightMotor.setPower(power);
        }
    }

    public void setArmPower(int encoderPos) {
        double pow = pid.calculate(encoderPos, robot.armLeftMotor.getCurrentPosition(), false);
        robot.armLeftMotor.setPower(pow);
        robot.armRightMotor.setPower(pow);
    }

    public void noEncoders() {
        robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
