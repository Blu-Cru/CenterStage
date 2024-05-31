package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PIDPoint {
    Pose2d pose;
    double translationTolerance;

    public PIDPoint(Pose2d pose, double translationTolerance) {
        this.pose = pose;
        this.translationTolerance = translationTolerance;
    }

    public PIDPoint(Pose2d pose) {
        this(pose, 1);
    }

    public boolean atTarget() {
        return Robot.getInstance().drivetrain.inRange(pose, translationTolerance);
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTranslationTolerance() {
        return translationTolerance;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public void setTranslationTolerance(double translationTolerance) {
        this.translationTolerance = translationTolerance;
    }
}
