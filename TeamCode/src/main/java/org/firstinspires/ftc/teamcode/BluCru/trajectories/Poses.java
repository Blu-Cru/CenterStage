package org.firstinspires.ftc.teamcode.BluCru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;

public class Poses {
    public static Pose2d closeStartingPose;
    public static Pose2d farStartingPose;

    public static Pose2d closePlacementFarPose;
    public static Pose2d closePlacementClosePose;
    public static Pose2d closePlacementCenterPose;

    public static Pose2d farPlacementFarPose;
    public static Pose2d farPlacementClosePose;
    public static Pose2d farPlacementCenterPose;

    public static Pose2d alignClosePose;

    public static Pose2d depositFarPose;
    public static Pose2d depositCenterPose;
    public static Pose2d depositClosePose;

    public static Pose2d closeParkPose;
    public static Pose2d farParkPose;

    private double reflect;

    public Poses(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            reflect = -1;
        } else {
            reflect = 1;
        }

        closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
        closePlacementFarPose = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
        closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
        closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

        farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
        farPlacementFarPose = new Pose2d(-44, -39 * reflect, Math.toRadians(-45 * reflect));
        farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
        farPlacementCenterPose = new Pose2d(-33, -31 * reflect, Math.toRadians(-90 * reflect));

        alignClosePose = new Pose2d(-60, -36*reflect, Math.toRadians(180));

        depositFarPose = new Pose2d(52, -29 * reflect, Math.toRadians(180));
        depositCenterPose = new Pose2d(52, -36 * reflect, Math.toRadians(180));
        depositClosePose = new Pose2d(52, -43 * reflect, Math.toRadians(180));

        closeParkPose = new Pose2d(60, -60 * reflect, Math.toRadians(180));
        farParkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));
    }
}
