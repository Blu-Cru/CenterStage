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
                        drive.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_CENTER_POSE)
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .setTangent(Math.toRadians(120 * reflect))
//                                // drop down, start intake, unlock
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                    robot.intake.dropToStack(4);
//                                })

                                .splineToLinearHeading(new Pose2d(Poses.STACK_X, -12 * reflect, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(INTAKE_TIME)
                                // lock and start outtaking

                                // stop outtake

                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .setTangent(Math.toRadians(0 * reflect))
                                .splineToConstantHeading(new Vector2d(-45, Poses.CENTER_Y * reflect), Math.toRadians(0))
                                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                                // lift

                                // wrist back

                                // turn turret

                                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
                                // release white pixel

                                // turn turret

                                // release yellow pixel

                                .waitSeconds(1.3)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}