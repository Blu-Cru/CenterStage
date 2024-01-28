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
                        drive.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CLOSE_POSE)
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .setTangent(Math.toRadians(270 * reflect))
                                .splineToConstantHeading(new Vector2d(30, -44 * reflect), 0)
                                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect, Math.toRadians(180)), 0)

//                                // lift
//                                .UNSTABLE_addTemporalMarkerOffset(LIFT_YELLOW_TIME, () -> {
//                                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
//                                })
//                                // wrist back
//                                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
//                                    robot.outtake.extendWrist();
//                                })

                                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), 0)

//                                // release
//                                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
//                                    robot.outtake.unlock();
//                                })
//                                // lift clear
//                                .UNSTABLE_addTemporalMarkerOffset(LIFT_CLEAR_TIME, () -> {
//                                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
//                                })

                                .waitSeconds(1.2)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}