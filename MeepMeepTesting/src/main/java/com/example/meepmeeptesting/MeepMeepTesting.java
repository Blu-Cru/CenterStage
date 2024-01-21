package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double reflect = 1;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(360), Math.toRadians(400), 12.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Poses.WING_STARTING_POSE)
                                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                                // placement
                                .setTangent(Math.toRadians(90 * reflect))
                                .splineTo(new Vector2d(-36, -60*reflect), Math.toRadians(90 * reflect))
                                .splineToSplineHeading(new Pose2d(-36, -38 * reflect, Math.toRadians(180)), Math.toRadians(90 * reflect))
                                .splineToConstantHeading(new Vector2d(-36, -24 * reflect), Math.toRadians(90 * reflect))
                                .splineToConstantHeading(Poses.WING_PLACEMENT_FAR_FOR_CENTER_POSE.vec(), Math.toRadians(270 * reflect))
                                // release purple pixel

                                // drop down and start intake

                                .waitSeconds(0.5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}