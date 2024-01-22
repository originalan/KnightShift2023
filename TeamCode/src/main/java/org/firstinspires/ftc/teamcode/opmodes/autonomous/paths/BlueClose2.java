package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

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

@Autonomous(name = "BlueClose2 (push both pixels, park)", group = "AUTO")
public class BlueClose2 extends AutoBase {

    private TrajectorySequence parkingTraj, waitingTraj1;

    public enum AutoState {
        PARKING,
        RELEASE_PIXELS,
        IDLE
    }

    public AutoState autoState = AutoState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        startingPose = new Pose2d(12, 63.75, Math.toRadians(0));
        PoseStorage.startingAutoPose = new Pose2d(12, 63.75, Math.toRadians(0)); // to prevent shadowing

        initialize(JVBoysSoccerRobot.AllianceType.BLUE);
        PoseStorage.AUTO_SHIFT_DEGREES = 90.0;

        drive.setPoseEstimate(startingPose);
        buildTrajectories();

        robot.deliveryArm.armState = DeliveryArm.ArmState.AT_REST;
        robot.intake.intakeState = Intake.IntakeState.OFF;
        telemetry.addLine("Blue, starting closer to backstage");
        telemetry.addLine("Parks, spits pixels into backstage");
        telemetry.update();

        waitForStart();

        autoState = AutoState.PARKING;
        drive.followTrajectorySequenceAsync(parkingTraj);

        if (opModeIsActive()) {

            while (opModeIsActive() && !isStopRequested()) {

                switch (autoState) {

                    case PARKING:
                        // robot is parking
                        if (!drive.isBusy()) {
                            autoState = AutoState.RELEASE_PIXELS;
                            robot.intake.intakeState = Intake.IntakeState.REVERSE;
                            drive.followTrajectorySequenceAsync(waitingTraj1);
                        }
                        break;
                    case RELEASE_PIXELS:
                        // robot is spitting out pixels
                        if (!drive.isBusy()) {
                            autoState = AutoState.IDLE;
                            robot.intake.intakeState = Intake.IntakeState.OFF;
                        }
                        break;
                    case IDLE:
                        // robot is idle after parking and spitting out pixels
                        break;

                }

                drive.update();
                robot.deliveryArm.update();
                robot.intake.update();

                transferPose();
                telemetry.update();

            }

        }

    }

    public void buildTrajectories() {

        parkingTraj = drive.trajectorySequenceBuilder(startingPose)
                .waitSeconds(1) // idk j wait a second j in case
                .back(45)
                .build();

        waitingTraj1 = drive.trajectorySequenceBuilder(parkingTraj.end())
                .waitSeconds(5)
                .build();

    }

}
