package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagLocalizer {
    public static Vector2d CAMERA_POS = new Vector2d(-6.61, 3.78); // position of the camera relative to the center of the robot in inches
    public static double TAG_X = 64.5; // x position of the tags in inches
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

    public static Vector2d getTagToRobotVector(Vector2d robotToTag, double detectionYawRad) {
        return robotToTag.rotated(-detectionYawRad);
    }

    public static Pose2d getRobotPose(int tagId, double detectionX, double detectionY, double detectionYawRad) {
        Vector2d robotToTag = getRobotToTagVector(detectionX, detectionY);
        Vector2d tagToRobot = getTagToRobotVector(robotToTag, detectionYawRad);
        return new Pose2d(TAGS[tagId].getX() + tagToRobot.getX(), TAGS[tagId].getY() + tagToRobot.getY(), TAGS[tagId].getHeading() - detectionYawRad);
    }

    public static Pose2d getRobotPose(AprilTagDetection detection) {
        return getRobotPose(detection.id, detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(detection.ftcPose.yaw));
    }

    public static Pose2d getRobotPose(List<AprilTagDetection> detections) {
        if(detections.size() == 0) {
            return new Pose2d(0, 0, 0);
        } else {
            AprilTagDetection closestDetection = detections.get(0);
            double closestDistance = Math.hypot(closestDetection.ftcPose.x, closestDetection.ftcPose.y);

            for (AprilTagDetection detection : detections) {
                double distance = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
                if(distance < closestDistance) {
                    closestDetection = detection;
                    closestDistance = distance;
                }
            }

            return getRobotPose(closestDetection);
        }
    }
}
