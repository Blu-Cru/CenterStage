package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.NoSuchElementException;

public class FusedLocalizer {
    public static double TAG_UPDATE_DELAY = 100; // ms between tag updates
    Localizer deadWheels;
    IMU imu;
    PoseHistory poseHistory;
    long lastFrameTime;
    double lastTagUpdateMillis;

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    double headingOffset = 0;
    double lastImuUpdateMillis = 0;
    public boolean usingIMU = true;

    YawPitchRollAngles ypr;

    public FusedLocalizer(Localizer localizer, HardwareMap hardwareMap) {
        deadWheels = localizer;
        poseHistory = new PoseHistory();

        imu = hardwareMap.get(IMU.class, "e hub imu");
        lastFrameTime = System.nanoTime();
        lastTagUpdateMillis = System.currentTimeMillis();
    }

    public void update() {
        // make a copy of the current pose, so that the pose history doesn't get updated with the same object
        deadWheels.update();
        Pose2d currentPose = deadWheels.getPoseEstimate();
        //Log.v("Marker Entry", "Pos" + currentPose);
        poseHistory.add(currentPose, deadWheels.getPoseVelocity());

        if(System.currentTimeMillis() - lastImuUpdateMillis > 100 && usingIMU) {
            lastImuUpdateMillis = System.currentTimeMillis();

            ypr = imu.getRobotYawPitchRollAngles();
            Log.v("FusedLocalizer", "Updating IMU, correction = " + (ypr.getYaw(AngleUnit.RADIANS) + headingOffset - deadWheels.getPoseEstimate().getHeading()));
            Pose2d currentPoseWithHeading = new Pose2d(currentPose.getX(), currentPose.getY(), Angle.norm(ypr.getYaw(AngleUnit.RADIANS) + headingOffset));
            deadWheels.setPoseEstimate(currentPoseWithHeading);
            deadWheels.update();
        }
    }

    public void updateAprilTags(AprilTagProcessor tagProcessor) {
        if(System.currentTimeMillis() - lastTagUpdateMillis < TAG_UPDATE_DELAY) return; // only update every TAG_UPDATE_DELAY ms

        Pose2d currentPose = deadWheels.getPoseEstimate();
        double heading = Angle.norm(currentPose.getHeading());
        if(heading < Math.PI/2 || heading > 3*Math.PI/2) {
            Log.e("FusedLocalizer", "Not updating tags, robot is facing the wrong way");
            return;
        }

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections.size() < 1) {
            Log.e("FusedLocalizer", "No tags found");
            return;
        }

        // get odo pose at the time of the tag pose
        long timeOfFrame = detections.get(0).frameAcquisitionNanoTime;
        if(timeOfFrame==lastFrameTime) {
            Log.i("FusedLocalizer", "Already updated with this frame");
            return;
        }
        PoseMarker poseMarkerAtFrame = poseHistory.getPoseAtTime(timeOfFrame);
        Pose2d poseAtFrame = poseMarkerAtFrame.pose;
        Pose2d velocityAtFrame = poseMarkerAtFrame.velocity;




        long timeSinceFrame = System.nanoTime() - timeOfFrame;
        Log.v("FusedLocalizer", "Time since frame:" + timeSinceFrame);

        // save reference to tag pose
        Pose2d tagPose;
        try {
            tagPose = AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(detections, poseAtFrame.getHeading());
        } catch(Exception e) {
            return;
        }

        double weight = getWeight(velocityAtFrame);
        Log.v("FusedLocalizer", "Weight: " + weight);

        if(weight == 0) {
            Log.e("FusedLocalizer", "Weight is 0, not updating");
            return;
        }

        // calculate change from old odo pose to current pose
        currentPose = deadWheels.getPoseEstimate();
        Pose2d odoDelta = currentPose.minus(poseAtFrame);

        Pose2d odoPoseError = tagPose.minus(poseAtFrame).times(weight);

        Log.v("FusedLocalizer", "Tag pose: " + tagPose);
        Log.v("FusedLocalizer", "Pose at frame:" + poseAtFrame);
        Log.v("FusedLocalizer", "Current pose: " + currentPose);
        Log.v("FusedLocalizer", "Correction: " + odoPoseError);


        Pose2d newPose = new Pose2d(tagPose.vec().plus(odoDelta.vec()), currentPose.getHeading());
        Log.i("FusedLocalizer", "Updated pose to: " + newPose);

        // set pose estimate to tag pose + delta
        deadWheels.setPoseEstimate(newPose);
        deadWheels.update();
        lastTagUpdateMillis = System.currentTimeMillis();
        // add tag - odo to pose history
        Log.v("FusedLocalizer", "odoPoseError: " + odoPoseError);
        poseHistory.offset(odoPoseError);
        lastFrameTime = timeOfFrame;
    }

    public void init() {
        imu.resetDeviceConfigurationForOpMode();
        imu.initialize(parameters);

        lastImuUpdateMillis = System.currentTimeMillis();
    }

    public void resetHeading(double newHeading) {
        newHeading = Angle.norm(newHeading);
        headingOffset = newHeading - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        deadWheels.setPoseEstimate(new Pose2d(deadWheels.getPoseEstimate().vec(), newHeading));
        deadWheels.update();
    }

    public double getWeight(Pose2d velocity) {
        double angVel = velocity.getHeading();
        double vel = velocity.vec().norm();

        double totalVel = Math.hypot(vel, angVel * 10);

        // TODO: tune this function
        return Range.clip(-0.3*Math.atan(.12 * totalVel-4), 0, 1);
    }
}
