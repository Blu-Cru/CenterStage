package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class AprilTagLocalizer {
    public static Vector2d CAMERA_POS = new Vector2d(-6.61, 3.78); // position of the camera relative to the center of the robot in inches
    public static double TAG_X = 60;
    public static Pose2d[] TAGS = {
            new Pose2d(0, 0, Math.toRadians(0)), // tag 0
            new Pose2d(TAG_X, 42, Math.toRadians(180)), // tag 1
            new Pose2d(TAG_X, 36, Math.toRadians(180)), // tag 2
            new Pose2d(TAG_X, 30, Math.toRadians(180)), // tag 3
            new Pose2d(TAG_X, -30, Math.toRadians(180)), // tag 4
            new Pose2d(TAG_X, -36, Math.toRadians(180)), // tag 5
            new Pose2d(TAG_X, -42, Math.toRadians(180)), // tag 6
    };

    public static Vector2d getRobotToTagVector(double detectionX, double detectionY) {
        double x = -detectionY + CAMERA_POS.getX();
        double y = detectionX + CAMERA_POS.getY();
        return new Vector2d(x, y);
    }

    public static Pose2d getRobotPose(int tagId, double detectionX, double detectionY, double detectionYaw) {
        Vector2d robotToTag = getRobotToTagVector(detectionX, detectionY);
        Vector2d tagToRobot = robotToTag.times(-1).rotated(-TAGS[tagId].getHeading());
        return new Pose2d(tagToRobot.getX(), tagToRobot.getY(), TAGS[tagId].getHeading() - detectionYaw);
    }
}
