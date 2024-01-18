package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FullStateFeedback;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

/**
 * DeliveryArm is a Subsystem representing all linear slide / arm hardware movement
 */
@Config
public class DeliveryArm extends Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;
    private JVBoysSoccerRobot robot;
    private FullStateFeedback controller;
    private InterpLUT kPCoefficients = new InterpLUT(),
                        kVCoefficients = new InterpLUT();

    public static double kPatTop = 0;
    public static double kVatTop = 0;
    public static double kPatRest = 0;
    public static double kVatRest = 0;

    public double targetPower = 0;

    public enum ArmState {
        AT_REST,
        GO_TO_POSITION,
        BOTTOM,
        TOP,
        AUTO_INTAKE_1,
        AUTO_INTAKE_2,
        TEST
    }

    public ArmState armState = ArmState.AT_REST;

    public DeliveryArm(HardwareMap hwMap, Telemetry telemetry, JVBoysSoccerRobot robot) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.robot = robot;
        controller = new FullStateFeedback(hwMap, telemetry);

//        robot.deliveryArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        kPCoefficients.add(0, kPatRest);
//        kPCoefficients.add(120, kPatTop); // change 120 degrees to ____ encoder ticks
//        kVCoefficients.add(0, kVatRest);
//        kVCoefficients.add(120, kVatTop);
//
//        kPCoefficients.createLUT();
//        kVCoefficients.createLUT();

    }

    @Override
    public void addTelemetry() {
        telemetry.addLine("Delivery Arm");
        telemetry.addData("   Motor Position Encoder Value", "%d", robot.deliveryArmMotor.getCurrentPosition());
    }

    @Override
    public void update() {
        switch (armState) {
//            case GO_TO_POSITION:
//                robot.deliveryArmMotor.setPower(targetPower);
//                break;
//            case BOTTOM:
//                robot.deliveryArmMotor.setPower(controller.calculate(0, 0, robot.deliveryArmMotor.getCurrentPosition(), robot.deliveryArmMotor.getVelocity()));
//                break;
//            case TOP:
//                robot.deliveryArmMotor.setPower(controller.calculate((int)(150.0/360.0 * 537.6), 0, robot.deliveryArmMotor.getCurrentPosition(), robot.deliveryArmMotor.getVelocity()));
//                break;
            case TEST:
//                robot.deliveryArmMotor.setTargetPosition(RobotSettings.ARM_ENCODER_TOP);
//                robot.deliveryArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.deliveryArmMotor.setPower(RobotSettings.ARM_MOTOR_POWER);
                break;
            case AT_REST:
                robot.deliveryArmMotor.setPower(0);
                break;
        }
    }

    @Override
    public void stop() {
        // Unextend the slide
    }

//    public void gainScheduling(double targetPosition, double targetVelocity) {
//
//        double armAngle = robot.deliveryArmMotor.getCurrentPosition() / 360.0; // actual position of arm -> degrees
//
////        double Kp = kPCoefficients.get(armAngle);
////        double Kv = kVCoefficients.get(armAngle);
//
//        controller.setCoefficients(Kp, Kv);
//        double output = controller.calculate(targetPosition, targetVelocity, robot.deliveryArmMotor.getCurrentPosition(), robot.deliveryArmMotor.getVelocity());
//        robot.deliveryArmMotor.setPower(output);
//
//    }

}
