package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.content.res.Resources;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;

public class FusedLocalizer {
    Localizer deadWheels;
    IMU imu;
    PoseHistory poseHistory;

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
    }

    public void update() {
        // make a copy of the current pose, so that the pose history doesn't get updated with the same object
        deadWheels.update();
        Pose2d currentPose = deadWheels.getPoseEstimate();
        poseHistory.add(currentPose);

        if(System.currentTimeMillis() - lastImuUpdateMillis > 100 && usingIMU) {
            lastImuUpdateMillis = System.currentTimeMillis();

            ypr = imu.getRobotYawPitchRollAngles();
            Log.v("FusedLocalizer", "Updating IMU, correction = " + (ypr.getYaw(AngleUnit.RADIANS) + headingOffset - deadWheels.getPoseEstimate().getHeading()));
            Pose2d currentPoseWithHeading = new Pose2d(currentPose.getX(), currentPose.getY(), Angle.norm(ypr.getYaw(AngleUnit.RADIANS) + headingOffset));
            deadWheels.setPoseEstimate(currentPoseWithHeading);
        }
    }

    public void updateAprilTags(AprilTagProcessor tagProcessor) {
        double heading = Angle.norm(deadWheels.getPoseEstimate().getHeading());
//        Log.v("FusedLocalizer", "heading: " + heading);
        if(heading < Math.PI/2 || heading > 3*Math.PI/2) throw new IllegalArgumentException("Not in the right orientation to update tags");

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections.size() < 1) throw new NoSuchElementException("No tags detected");

        // get odo pose at the time of the tag pose
        long timeOfFrame = detections.get(0).frameAcquisitionNanoTime;
        Pose2d odoPoseTimeOfFrame = poseHistory.getPoseAtTime(timeOfFrame);

        long timeSinceFrame = System.nanoTime() - timeOfFrame;
        Log.v("FusedLocalizer", "Time since frame:" + timeSinceFrame);

        // save reference to tag pose
        Pose2d tagPoseTimeOfFrame = AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(detections, odoPoseTimeOfFrame.getHeading());

        // calculate change from old odo pose to current pose
        Pose2d odoDeltaPoseFrameToNow = deadWheels.getPoseEstimate().minus(odoPoseTimeOfFrame);

        Log.v("FusedLocalizer", "Updating pose. history odo pose: " + odoPoseTimeOfFrame.toString() + " tag pose: " + tagPoseTimeOfFrame);
        Log.v("FusedLocalizer", "tag pose: " + tagPoseTimeOfFrame);
        Log.v("FusedLocalizer", "delta pose: " + odoDeltaPoseFrameToNow);


        Pose2d newPose = new Pose2d(tagPoseTimeOfFrame.vec().plus(odoDeltaPoseFrameToNow.vec()), deadWheels.getPoseEstimate().getHeading());
        Log.v("FusedLocalizer", "new pose: " + newPose);

        // set pose estimate to tag pose + delta
        deadWheels.setPoseEstimate(newPose);
//        deadWheels.setPoseEstimate(tagPoseTimeOfFrame.plus(odoDeltaPoseFrameToNow));
        // add tag - odo to pose history
        Pose2d odoPoseError = tagPoseTimeOfFrame.minus(odoPoseTimeOfFrame);
        poseHistory.offset(odoPoseError);
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
    }

    public double getWeight() {
        return 1.0;
    }
}
