package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FullStateFeedback;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * DeliveryArm is a Subsystem representing all delivery arm hardware movement
 */
@Config
public class DeliveryArm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private PIDFControl pid;

    public double targetPower = 0;
    public double projectedPower = 0;
    public boolean overridePowerForward = false;
    public boolean overridePowerBackward = false;

    public enum ArmState {
        AT_REST,
        BOTTOM,
        TOP,
        LIFT, // lifts arm up a little bit, used for purple pixel placement
        TEST,
        GO_TO_POSITION
    }

    public ArmState armState = ArmState.AT_REST;

    public DeliveryArm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        this.pid = new PIDFControl();

        JVBoysSoccerRobot.initialArmPosition = robot.deliveryArmMotor.getCurrentPosition();
    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Delivery Arm");
        telemetry.addData("   Motor Position Encoder Value", "%d", robot.deliveryArmMotor.getCurrentPosition());
    }

    @Override
    public void update() {
        switch (armState) {
            case GO_TO_POSITION:
                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                setArmPower(targetPower, overridePowerForward, overridePowerBackward);
                break;
            case BOTTOM:
                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                projectedPower = pid.calculate(JVBoysSoccerRobot.initialArmPosition + 3,
                        robot.deliveryArmMotor.getCurrentPosition(),
                        false);
                setArmPower(projectedPower, overridePowerForward, overridePowerBackward);
                break;
            case LIFT: // lifts arm a little bit
                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                projectedPower = pid.calculate(JVBoysSoccerRobot.initialArmPosition + 90,
                        robot.deliveryArmMotor.getCurrentPosition(),
                        false);
                setArmPower(projectedPower, overridePowerForward, overridePowerBackward);
                break;
            case TOP:
                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                projectedPower = pid.calculate((int)(JVBoysSoccerRobot.initialArmPosition + (150.0 / 360.0 * 537.6 * 3)),
                        robot.deliveryArmMotor.getCurrentPosition(),
                        false);
                setArmPower(projectedPower, overridePowerForward, overridePowerBackward);
                break;
            case TEST:
                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                setArmPower(0.8, overridePowerForward, overridePowerBackward);
                break;
            case AT_REST:
                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                projectedPower = 0;
                robot.deliveryArmMotor.setPower(0);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void setArmPower(double power, boolean overrideForward, boolean overrideBackward) {
        if (overrideForward) {
            robot.deliveryArmMotor.setPower(power - RobotSettings.ARM_OVERRIDE_POWER);
        }else if (overrideBackward) {
            robot.deliveryArmMotor.setPower(power + RobotSettings.ARM_OVERRIDE_POWER);
        }else {
            robot.deliveryArmMotor.setPower(power);
        }
    }

}
