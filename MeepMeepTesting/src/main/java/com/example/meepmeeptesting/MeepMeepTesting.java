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
                .setDimensions(14.3,14.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                                .setTangent(Math.toRadians(135 * reflect))

//                                // retract turret
//                                .UNSTABLE_addTemporalMarkerOffset(CENTER_TURRET_TIME, () -> robot.outtake.centerTurret())
//                                // retract wrist
//                                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_TIME, () -> robot.outtake.retractWrist())
//                                // retract lift
//                                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_TIME, () -> {
//                                    robot.outtake.retractLift();
//                                    robot.intake.intakeWrist.dropToAutoMidPos();
//                                })

                                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-30 + Poses.FIELD_OFFSET_X, -12 * reflect), Math.toRadians(180))
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X + Poses.FIELD_OFFSET_X, -8 * reflect), Math.toRadians(180))
//                                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
//                                    robot.intake.dropToStack(stackHeight);
//                                    robot.intake.intake();
//                                    robot.outtake.unlock();
//                                    robot.intakingInAuto = true;
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                    robot.intake.dropToGround();
//                                })
                                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                                .splineToConstantHeading(new Vector2d(Poses.STACK_X + Poses.FIELD_OFFSET_X, -20 * reflect), Math.toRadians(270 * reflect))
                                .waitSeconds(1)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}