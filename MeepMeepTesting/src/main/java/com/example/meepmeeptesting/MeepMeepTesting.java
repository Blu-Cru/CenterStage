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
                        drive.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                                .setTangent(Math.toRadians(135 * reflect))
                                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .splineToConstantHeading(new Vector2d(-54, -8 * reflect), Math.toRadians(180 * reflect))
                                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -16 * reflect), Math.toRadians(-90 * reflect))
                                .splineToConstantHeading(new Vector2d(-60, -8 * reflect), Math.toRadians(90 * reflect))
                                // intake ready

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}