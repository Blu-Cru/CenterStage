package org.firstinspires.ftc.teamcode.blucru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.states.Alliance;

public class Poses {
    public static double BACKDROP_X = 52;
    public static double STACK_X = -58;
    public static double START_Y = -62;

    public static Pose2d BACKDROP_STARTING_POSE;
    public static Pose2d WING_STARTING_POSE;

    public static Pose2d BACKDROP_PLACEMENT_FAR_POSE;
    public static Pose2d BACKDROP_PLACEMENT_CLOSE_POSE;
    public static Pose2d BACKDROP_PLACEMENT_CENTER_POSE;

    public static Pose2d WING_PLACEMENT_FAR_POSE;
    public static Pose2d WING_PLACEMENT_CLOSE_POSE;
    public static Pose2d WING_PLACEMENT_CENTER_POSE;

    public static Pose2d STACK_CLOSE_POSE;
    public static Pose2d STACK_CENTER_POSE;
    public static Pose2d STACK_FAR_POSE;

    public static Pose2d DEPOSIT_FAR_POSE;
    public static Pose2d DEPOSIT_CENTER_POSE;
    public static Pose2d DEPOSIT_CLOSE_POSE;

    public static Pose2d PARK_CLOSE_POSE;
    public static Pose2d PARK_FAR_POSE;

    public static double reflect = 1;

    public Poses(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            reflect = -1;
        } else {
            reflect = 1;
        }

        BACKDROP_STARTING_POSE = new Pose2d(12, START_Y * reflect, Math.toRadians(-90 * reflect));
        BACKDROP_PLACEMENT_FAR_POSE = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
        BACKDROP_PLACEMENT_CLOSE_POSE = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
        BACKDROP_PLACEMENT_CENTER_POSE = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

        WING_STARTING_POSE = new Pose2d(-36, START_Y * reflect, Math.toRadians(-90 * reflect));
        WING_PLACEMENT_FAR_POSE = new Pose2d(-44, -39 * reflect, Math.toRadians(-45 * reflect));
        WING_PLACEMENT_CLOSE_POSE = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
        WING_PLACEMENT_CENTER_POSE = new Pose2d(-33, -31 * reflect, Math.toRadians(-90 * reflect));



        DEPOSIT_FAR_POSE = new Pose2d(52, -29 * reflect, Math.toRadians(180));
        DEPOSIT_CENTER_POSE = new Pose2d(52, -36 * reflect, Math.toRadians(180));
        DEPOSIT_CLOSE_POSE = new Pose2d(52, -43 * reflect, Math.toRadians(180));

        PARK_CLOSE_POSE = new Pose2d(60, -60 * reflect, Math.toRadians(180));
        PARK_FAR_POSE = new Pose2d(60, -12 * reflect, Math.toRadians(180));
    }
}
