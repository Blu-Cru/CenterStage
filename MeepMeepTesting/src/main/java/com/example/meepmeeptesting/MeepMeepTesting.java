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
                        drive.trajectorySequenceBuilder(new Pose2d(Poses.STACK_SETUP_X, -12 * reflect, Math.toRadians(180)))
                                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                                .setTangent(Math.toRadians(0 * reflect))
                                // drop to intake, start intake, unlock
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                                    robot.intake.dropToStack(3);
//                                    robot.intake.setIntakePower(1);
//                                    robot.outtake.unlock();
//                                })

                                .splineToConstantHeading(new Vector2d(-10 + Poses.FIELD_OFFSET_X, Poses.CENTER_Y * reflect), Math.toRadians(0))
                                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))
//
//                                // lift
//                                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
//                                    robot.outtake.lift.setMotionProfileTargetPos(670);
//                                })
//                                // wrist back
//                                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
//                                    robot.outtake.extendWrist();
//                                })
//                                // turn turret
//                                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
//                                    robot.outtake.setTurretAngle(270 - 50 * reflect);
//                                })

                                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
//                                // release white pixel
//                                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
//                                    robot.outtake.unlockFrontLockBack();
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                                    robot.outtake.lift.setMotionProfileTargetPos(900);
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                    robot.outtake.lockFront();
//                                })
//                                // turn turret
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
//                                    robot.outtake.setTurretAngle(270 + 25 * reflect);
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//                                    robot.outtake.lift.setMotionProfileTargetPos(670);
//                                })
//                                // release yellow pixel
//                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    robot.outtake.unlock();
//                                })
//                                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
//                                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
//                                })
//                                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}