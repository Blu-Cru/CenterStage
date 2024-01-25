package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double reflect = 1;

    public static double DEPOSIT_TIME = 1.2;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(360), Math.toRadians(400), 12.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                                .setTangent(Math.toRadians(225 * reflect))
                                // retract turret

                                // retract wrist

                                // retract lift

                                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-40, -60 * reflect), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                                // start and lower intake

                                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                                .waitSeconds(0.5)
                                // lock and raise intake, start outtaking

                                // stop outtaking

                                .setTangent(Math.toRadians(-45 * reflect))
                                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                                // lift

                                // wrist back

                                .setConstraints(Constraints.SLOW_VELOCITY, Constraints.SLOW_ACCELERATION)
                                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                                // release pixel

                                .waitSeconds(DEPOSIT_TIME)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}