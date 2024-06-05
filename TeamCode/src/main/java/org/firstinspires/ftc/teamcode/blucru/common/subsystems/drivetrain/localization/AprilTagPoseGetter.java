package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.List;

public final class AprilTagPoseGetter {
    public static Vector2d CAMERA_POS = new Vector2d(-6.61, 3.78); // position of the camera relative to the center of the robot in inches
    public static double
            TAG_X = 62,
            MAX_UPDATE_DISTANCE = 35; // maximum update distance
    public static HashMap<Integer, Pose2d> TAGS = new HashMap<Integer, Pose2d>() {{
        put(1, new Pose2d(TAG_X, 42, Math.toRadians(180))); // tag 1 (red right)
        put(2, new Pose2d(TAG_X, 36, Math.toRadians(180))); // tag 2 (red center)
        put(3, new Pose2d(TAG_X, 30, Math.toRadians(180))); // tag 3 (red left)
        put(4, new Pose2d(TAG_X, -30, Math.toRadians(180))); // tag 4 (blue right)
        put(5, new Pose2d(TAG_X, -36, Math.toRadians(180))); // tag 5 (blue center)
        put(6, new Pose2d(TAG_X, -42, Math.toRadians(180))); // tag 6 (blue left)
    }};

    public static Vector2d getRobotToTagVector(double detectionX, double detectionY) {
        //
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
        Pose2d tagPose = TAGS.get(tagId);

        return new Pose2d(tagPose.vec().plus(tagToRobot), tagPose.getHeading() - detectionYawRad);
    }

    public static Pose2d getRobotPoseWithHeading(int tagId, double detectionX, double detectionY, double heading) {
        Vector2d robotToTag = getRobotToTagVector(detectionX, detectionY);
        Vector2d globalTagToRobot = robotToTag.rotated(heading).unaryMinus();
        Pose2d tagPose = TAGS.get(tagId);

        return new Pose2d(tagPose.vec().plus(globalTagToRobot), heading);
    }

    public static Pose2d getRobotPose(AprilTagDetection detection) {
        return getRobotPose(detection.id, detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(detection.ftcPose.yaw));
    }

    public static Pose2d getRobotPoseWithHeading(AprilTagDetection detection, double heading) {
        return getRobotPoseWithHeading(detection.id, detection.ftcPose.x, detection.ftcPose.y, Angle.norm(heading));
    }

    public static Pose2d getRobotPoseAtTimeOfFrame(List<AprilTagDetection> detections) {
        if(detections.size() == 0) {
            throw new NullPointerException("No tags found");
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

            if(closestDistance > MAX_UPDATE_DISTANCE) {
                throw new IllegalStateException("Too far away to update tags");
            }

            return getRobotPose(closestDetection);
        }
    }

    public static Pose2d getRobotPoseAtTimeOfFrame(List<AprilTagDetection> detections, double heading) {
        if(detections.size() == 0) {
            return null;
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

            if(closestDistance > MAX_UPDATE_DISTANCE) {
                return null;
            }

            Pose2d poseWithHeading = getRobotPoseWithHeading(closestDetection, heading);

            Log.v("TagPoseGetter", "got pose with heading: " + poseWithHeading);
            return poseWithHeading;
        }
    }
}
