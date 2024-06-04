package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;
import java.util.HashMap;

public class PIDPathBuilder {
    private final ArrayList<PathSegment> segments;
    private HashMap<Integer, ArrayList<Command>> commands; // commands to run before the point of the same index is reached

    public PIDPathBuilder() {
        segments = new ArrayList<PathSegment>();
    }

    public PIDPathBuilder addPoint(PIDPointSegment point) {
        segments.add(point);
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, double translationTolerance) {
        segments.add(new PIDPointSegment(pose, translationTolerance));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose) {
        segments.add(new PIDPointSegment(pose));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, boolean stopRequired) {
        segments.add(new PIDPointSegment(pose, stopRequired));
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose, double translationTolerance, boolean stopRequired) {
        segments.add(new PIDPointSegment(pose, translationTolerance, stopRequired));
        return this;
    }

    public PIDPathBuilder schedule(Command command) {
        commands.computeIfAbsent(segments.size(), k -> new ArrayList<Command>());
        commands.get(segments.size()).add(command);
        return this;
    }

    public PIDPathBuilder setPower(double power) {
        return schedule(new InstantCommand(() -> {
            Robot.getInstance().drivetrain.drivePower = power;
        }));
    }

    public PIDPathBuilder waitMillis(double milliseconds) {
        segments.add(new WaitSegment(segments.get(segments.size() - 1).getPose(), milliseconds));
        return this;
    }

    public PIDPath build() {
        return new PIDPath(segments, commands);
    }
}
