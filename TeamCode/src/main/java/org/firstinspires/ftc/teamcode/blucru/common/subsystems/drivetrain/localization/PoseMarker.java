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
//        Log.v("PoseMarker", "Created new PoseMarker at pose: " + this.pose + ", pose hash code:" + this.pose.hashCode());
    }

    public PoseMarker(long nanoTime, Pose2d pose) {
        this.nanoTime = nanoTime;
        this.pose = new Pose2d(new Vector2d(pose.getX(), pose.getY()), pose.getHeading());
    }
}
