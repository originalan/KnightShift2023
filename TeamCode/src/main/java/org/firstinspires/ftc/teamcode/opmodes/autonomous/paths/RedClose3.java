package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import static org.firstinspires.ftc.teamcode.util.RobotSettings.AUTO_PURPLE_PIXEL_RELEASE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DeliveryArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.JVBoysSoccerRobot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.RobotSettings;

@Autonomous(name = "RedClose3 (ignore team prop, with arm, parks)", group = "AUTO")
public class RedClose3 extends AutoBase {

    private TrajectorySequence backboardTraj, parkingTraj, waitingTraj1,
            waitingTraj3, waitingTraj2;

    public enum AutoState {
        MOVING_TO_BACKBOARD,
        POSITION_ARM,
        RELEASE_PIXEL,
        BRING_ARM_DOWN,
        PARKING,
        IDLE
    }

    public AutoState autoState = AutoState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(12, -63.125, Math.toRadians(90)); // the front of the robot is the control hub, not the delivery box so we rotate this by 180 degrees
        PoseStorage.startingAutoPose = new Pose2d(12, -63.125, Math.toRadians(90)); // to prevent shadowing

        initialize(JVBoysSoccerRobot.AllianceType.RED);
        PoseStorage.AUTO_SHIFT_DEGREES = 0;

        drive.setPoseEstimate(startingPose);
        buildTrajectories();

        while (opModeInInit()) {
            telemetry.addLine("Red, starting closer to backstage");
            telemetry.addLine("Drops both pixels onto backdrop, parks");
            telemetry.update();
        }

        waitForStart();

        autoState = AutoState.MOVING_TO_BACKBOARD;
        drive.followTrajectorySequenceAsync(backboardTraj);

        if (opModeIsActive()) {

            while (opModeIsActive() && !isStopRequested()) {

                switch (autoState) {

                    case MOVING_TO_BACKBOARD:
                        // robot is moving to the backboard
                        if (!drive.isBusy()) {
                            autoState = AutoState.POSITION_ARM;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case POSITION_ARM:
                        // At backboard, moving arm in place
                        robot.deliveryArm.armState = DeliveryArm.ArmState.TOP;
                        if (!drive.isBusy() && withinMargin(robot.deliveryArmMotor.getCurrentPosition(), RobotSettings.ARM_ENCODER_TOP, 10)) {
                            robot.intake.intakeState = Intake.IntakeState.REVERSE;
                            autoState = AutoState.RELEASE_PIXEL;
                            drive.followTrajectorySequenceAsync(waitingTraj2);
                        }
                    case RELEASE_PIXEL:
                        // robot is at backdrop, arm is in place, intake is in place, wait 3 seconds as intake reverses to release pixel
                        if (!drive.isBusy()) {
                            robot.deliveryArm.armState = DeliveryArm.ArmState.BOTTOM; // bring arm back down
                            robot.intake.intakeState = Intake.IntakeState.OFF; // turn off intake
                            autoState = AutoState.BRING_ARM_DOWN;
                            drive.followTrajectorySequenceAsync(waitingTraj3);
                        }
                        break;
                    case BRING_ARM_DOWN:
                        // robot is still unmoving, giving 5 seconds to bring arm back down
                        if (!drive.isBusy() && withinMargin(robot.deliveryArmMotor.getCurrentPosition(), 3, 10)) {
                            autoState = AutoState.BRING_ARM_DOWN;
                            drive.followTrajectorySequenceAsync(parkingTraj);
                        }
                        break;
                    case PARKING:
                        // robot is moving to parking destination
                        if (!drive.isBusy()) {
                            autoState = AutoState.IDLE;
                        }
                        break;
                    case IDLE:
                        break;

                }

                drive.update();
                robot.deliveryArm.update();
                robot.intake.update();

                transferPose();
                telemetry.update();

            }
//            drive.followTrajectorySequence(detectionTraj);

        }

    }


    public void buildTrajectories() {

        backboardTraj = drive.trajectorySequenceBuilder(startingPose)
                .splineTo(new Vector2d(60.75 - 8.875 - 2, -36), Math.toRadians(0))
                .build();

        parkingTraj = drive.trajectorySequenceBuilder(waitingTraj3.end())
                .forward(2)
                .build();


        waitingTraj1 = drive.trajectorySequenceBuilder(backboardTraj.end())
                .waitSeconds(5)
                .build();

        waitingTraj2 = drive.trajectorySequenceBuilder(waitingTraj1.end())
                .waitSeconds(3)
                .build();

        waitingTraj3 = drive.trajectorySequenceBuilder(waitingTraj2.end())
                .waitSeconds(5)
                .build();

    }

    public boolean withinMargin(double first, double second, double margin) {

        if (Math.abs(first - second) <= margin) {
            return true;
        }
        return false;

    }

}
