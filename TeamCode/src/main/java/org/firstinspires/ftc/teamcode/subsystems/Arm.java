package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDFControl;
import org.firstinspires.ftc.teamcode.util.RobotSettings;
import org.firstinspires.ftc.teamcode.util.UseTelemetry;

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

    public static int positionBottom = 0;
    public static int position1 = 50;
    public static int position2 = 100;
    public static int position3 = 150;

    public enum ArmState {
        AT_REST, // no power
        BOTTOM,
        GO_TO_POSITION,
        AUTO_POS,
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
        if (UseTelemetry.ARM) {
            telemetry.addLine("Arm");
            telemetry.addData("   Motor Position Encoder Value", "%d", robot.armLeftMotor.getCurrentPosition());
        }
    }

    @Override
    public void update() {
        switch (armState) {
            case GO_TO_POSITION:
                noEncoders();
                setArmPower(targetPower);
                break;
            case AT_REST:
                noEncoders();
                setArmPower(0);
                break;
            case NOTHING:
                break;
            case BOTTOM:
                robot.clawPivotServo.setPosition(RobotSettings.ARM_PIVOT_REST);
                setArmEncoderPosition(RobotSettings.ARM_BOTTOM_POSITION);
                break;
            case AUTO_POS:
                robot.clawPivotServo.setPosition(0.5);
                setArmEncoderPosition( (int)(200.0 / 360.0 * 537.6) + RobotSettings.ARM_BOTTOM_POSITION);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void setArmPower(double power) {
        robot.armLeftMotor.setPower(power);
        robot.armRightMotor.setPower(power);
    }

    public void setArmEncoderPosition(int encoderPos) {
        double pow = pid.calculate(encoderPos, robot.armLeftMotor.getCurrentPosition(), false);
        robot.armLeftMotor.setPower(pow);
        robot.armRightMotor.setPower(pow);
    }

    public void noEncoders() {
        robot.armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
