package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Poses {
    public static double BACKDROP_X = 52;
    public static double BACKDROP_SETUP_X = 48;

    public static double BACKDROP_FAR_Y = -31;
    public static double BACKDROP_CLOSE_Y = -41;

    public static double STACK_X = -58;
    public static double CENTER_Y = -10;
    public static double STACK_SETUP_X = -54;
    public static double START_Y = -62;

    public static Pose2d BACKDROP_STARTING_POSE;
    public static Pose2d WING_STARTING_POSE;

    public static Pose2d BACKDROP_PLACEMENT_FAR_POSE;
    public static Pose2d BACKDROP_PLACEMENT_CLOSE_POSE;
    public static Pose2d BACKDROP_PLACEMENT_CENTER_POSE;

    public static Pose2d WING_PLACEMENT_FAR_FOR_PERIM_POSE;
    public static Pose2d WING_PLACEMENT_FAR_FOR_CENTER_POSE;
    public static Pose2d WING_PLACEMENT_CLOSE_FOR_PERIM_POSE;
    public static Pose2d WING_PLACEMENT_CLOSE_FOR_CENTER_POSE;
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

    static {
        BACKDROP_STARTING_POSE = new Pose2d(12, START_Y * reflect, Math.toRadians(-90 * reflect));
        BACKDROP_PLACEMENT_FAR_POSE = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
        BACKDROP_PLACEMENT_CLOSE_POSE = new Pose2d(23.5, -35 * reflect, Math.toRadians(-90 * reflect));
        BACKDROP_PLACEMENT_CENTER_POSE = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

        WING_STARTING_POSE = new Pose2d(-36, START_Y * reflect, Math.toRadians(-90 * reflect));
        WING_PLACEMENT_FAR_FOR_PERIM_POSE = new Pose2d(-55, -36 * reflect, Math.toRadians(180));
        WING_PLACEMENT_FAR_FOR_CENTER_POSE = new Pose2d(-49.5, -19 * reflect, Math.toRadians(120));
        WING_PLACEMENT_CLOSE_FOR_CENTER_POSE = new Pose2d(-31, -34 * reflect, Math.toRadians(180));
        WING_PLACEMENT_CLOSE_FOR_PERIM_POSE = new Pose2d(-30, -40 * reflect, Math.toRadians(225 * reflect));
        WING_PLACEMENT_CENTER_POSE = new Pose2d(-50, -24.5 * reflect, Math.toRadians(180));

        DEPOSIT_FAR_POSE = new Pose2d(BACKDROP_X, BACKDROP_FAR_Y * reflect, Math.toRadians(180));
        DEPOSIT_CENTER_POSE = new Pose2d(BACKDROP_X, -36 * reflect, Math.toRadians(180));
        DEPOSIT_CLOSE_POSE = new Pose2d(BACKDROP_X, BACKDROP_CLOSE_Y * reflect, Math.toRadians(180));

        PARK_CLOSE_POSE = new Pose2d(48, -60 * reflect, Math.toRadians(180));
        PARK_FAR_POSE = new Pose2d(48, -12 * reflect, Math.toRadians(180));
    }
}
