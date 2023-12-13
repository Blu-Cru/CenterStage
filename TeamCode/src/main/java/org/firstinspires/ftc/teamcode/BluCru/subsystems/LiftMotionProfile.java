package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftMotionProfile {
    public double maxVelocity;
    public double maxAcceleration;
    public int targetPosition ;
    public int initialPosition;
    public double flip;
    double t1, t2, t3;
    double d1, d2, d3;

    public LiftMotionProfile(int targetPosition, int initialPosition) {
        this.targetPosition = targetPosition;
        this.initialPosition = initialPosition;
        this.maxVelocity = 700;
        this.maxAcceleration = 700;
        if(targetPosition < initialPosition) {
            flip = -1;
        } else {
            flip = 1;
        }
    }

    public LiftMotionProfile(int targetPosition, int initialPosition, double maxVelocity, double maxAcceleration) {
        this.targetPosition = targetPosition;
        this.initialPosition = initialPosition;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        if(targetPosition < initialPosition) {
            flip = -1;
        } else {
            flip = 1;
        }
    }

    public int calculateTargetPosition(double time) {
        t1 = maxVelocity / maxAcceleration;
        d1 = 0.5 * maxAcceleration * t1 * t1;

        double distance = Math.abs(targetPosition - initialPosition);
        double halfDistance = Math.abs(targetPosition - initialPosition) / 2.0;

        if(d1 > halfDistance) {
            t1 = Math.sqrt(2 * halfDistance / maxAcceleration);
            d1 = halfDistance;
        }
        d2 = distance - 2 * d1;
        t2 = d2 / maxVelocity;

        t3 = t1;

        if(time < t1) {
            return (int) ((0.5 * maxAcceleration * time * time) * flip + initialPosition);
        } else if(time < t1 + t2) {
            return (int) ((0.5 * maxAcceleration * t1 * t1 + maxVelocity * (time - t1)) * flip + initialPosition);
        } else if(time < t1 + t2 + t3) {
            return (int) ((0.5 * maxAcceleration * t1 * t1 + maxVelocity * t2 + maxVelocity * (time - t1 - t2) - 0.5 * maxAcceleration * (time - t1 - t2) * (time - t1 - t2)) * flip + initialPosition);
        } else {
            return targetPosition;
        }
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("t1", t1);
        telemetry.addData("t2", t2);
        telemetry.addData("t3", t3);
        telemetry.addData("d1", d1);
        telemetry.addData("d2", d2);
        telemetry.addData("d3", d3);
        telemetry.addData("target position", targetPosition);
        telemetry.addData("initial position", initialPosition);
        telemetry.addData("max velocity", maxVelocity);
        telemetry.addData("max acceleration", maxAcceleration);
    }
}
