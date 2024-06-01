package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class PIDPathBuilder {
    private final ArrayList<PIDPoint> points;

    public PIDPathBuilder() {
        points = new ArrayList<PIDPoint>();
    }

    public PIDPathBuilder addPoint(PIDPoint point) {
        points.add(point);
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, double translationTolerance) {
        points.add(new PIDPoint(pose, translationTolerance));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose) {
        points.add(new PIDPoint(pose));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, boolean stopRequired) {
        points.add(new PIDPoint(pose, stopRequired));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, double translationTolerance, boolean stopRequired) {
        points.add(new PIDPoint(pose, translationTolerance, stopRequired));
        return this;
    }

    public PIDPath build() {
        return new PIDPath(points);
    }
}
