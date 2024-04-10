package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagLocalizer {
    public static Vector2d CAMERA_POS = new Vector2d(-6.61, 3.78); // position of the camera relative to the center of the robot in inches
    public static double TAG_X = 62; // x position of the tags in inches
    public static Pose2d[] TAGS = {
            new Pose2d(0, 0, Math.toRadians(0)), // tag 0 (nothing)
            new Pose2d(TAG_X, 42, Math.toRadians(180)), // tag 1 (red right)
            new Pose2d(TAG_X, 36, Math.toRadians(180)), // tag 2 (red center)
            new Pose2d(TAG_X, 30, Math.toRadians(180)), // tag 3 (red left)
            new Pose2d(TAG_X, -30, Math.toRadians(180)), // tag 4 (blue right)
            new Pose2d(TAG_X, -36, Math.toRadians(180)), // tag 5 (blue center)
            new Pose2d(TAG_X, -42, Math.toRadians(180)), // tag 6 (blue left)
    };

    public static Vector2d getRobotToTagVector(double detectionX, double detectionY) {
        double x = -detectionY + CAMERA_POS.getX();
        double y = detectionX + CAMERA_POS.getY();
        return new Vector2d(x, y);
    }

    public static Vector2d getTagToRobotVector(Vector2d robotToTag, double detectionYaw) {
        return robotToTag.rotated(-detectionYaw);
    }

    public static Pose2d getRobotPose(int tagId, double detectionX, double detectionY, double detectionYaw) {
        Vector2d robotToTag = getRobotToTagVector(detectionX, detectionY);
        Vector2d tagToRobot = getTagToRobotVector(robotToTag, detectionYaw);
        return new Pose2d(TAGS[tagId].getX() + tagToRobot.getX(), TAGS[tagId].getY() + tagToRobot.getY(), TAGS[tagId].getHeading() - detectionYaw);
    }

    public static Pose2d getRobotPose(AprilTagDetection detection) {
        return getRobotPose(detection.id, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
    }
}
