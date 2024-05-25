package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseMarker {
    long nanoTime;
    Pose2d pose;

    public PoseMarker(Pose2d pose) {
        nanoTime = System.nanoTime();
        this.pose = new Pose2d(new Vector2d(pose.getX(), pose.getY()), pose.getHeading());
        this.log("PoseMarker created");
    }

    public PoseMarker(long nanoTime, Pose2d pose) {
        this.nanoTime = nanoTime;
        this.pose = new Pose2d(new Vector2d(pose.getX(), pose.getY()), pose.getHeading());
    }

    public void log(String tag) {
        Log.v(tag,  "PoseMarker at pose: " + pose + ", Pose hash code:" + pose.hashCode() + ", Time: " + nanoTime);
    }
}
