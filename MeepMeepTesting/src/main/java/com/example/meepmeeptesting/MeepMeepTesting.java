package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double reflect = 1;

    public static double DEPOSIT_TIME = 1.2;
    public static double INTAKE_TIME = 0.5;
    public static double RELEASE_TIME = 0.1;
    public static double WAIT_TIME = 0.1;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(360), Math.toRadians(400), 12.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                                .setTangent(Math.toRadians(90 * reflect))
                                .splineToConstantHeading(new Vector2d(-36, -52 * reflect), Math.toRadians(90 * reflect))
                                .splineToSplineHeading(Poses.WING_PLACEMENT_CLOSE_FOR_CENTER_POSE, Math.toRadians(45 * reflect))
                                // release purple pixel

                                .waitSeconds(WAIT_TIME)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}