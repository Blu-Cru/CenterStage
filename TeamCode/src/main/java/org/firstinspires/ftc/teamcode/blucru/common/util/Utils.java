package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;

public final class Utils {
    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }

    public static Pose2d mapPose(double x, double y, double heading) {
        return new Pose2d(x, y * Poses.reflect, heading * Poses.reflect);
    }
}
