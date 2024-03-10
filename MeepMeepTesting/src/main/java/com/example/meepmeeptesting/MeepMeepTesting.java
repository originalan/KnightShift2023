package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d startingPose;
        double x = 12;
        double y = 54.3;
        double heading = Math.toRadians(270);
        startingPose = new Pose2d(x, y, heading);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(130), Math.toRadians(130), 17.67)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPose)
                                .splineTo(new Vector2d(23 - 1.725, (-36.0 - 2.25 + 1) * -1), Math.toRadians(270))
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11, (-49.5 + 20.25 + 1.725 - 12.0 + 8) * -1), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(1)
                                .back(5.5)
                                .waitSeconds(1)
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(60.75 - 32.5 - 0.25 + 11 + 5.5, 50))
                                .waitSeconds(1)
                                .turn(Math.toRadians(-90))
                                .back(17)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}