package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LiftMotionProfile {
    public double maxVelocity;
    public double maxAcceleration;
    public int targetPosition ;
    public int initialPosition;

    public LiftMotionProfile(int targetPosition, int initialPosition) {
        this.targetPosition = targetPosition;
        this.initialPosition = initialPosition;
        this.maxVelocity = 100;
        this.maxAcceleration = 100;
    }

    public LiftMotionProfile(int targetPosition, int initialPosition, double maxVelocity, double maxAcceleration) {
        this.targetPosition = targetPosition;
        this.initialPosition = initialPosition;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public int calculateTargetPosition(double time) {
        double t1 = maxVelocity / maxAcceleration;
        double d1 = 0.5 * maxAcceleration * t1 * t1;

        double distance = Math.abs(targetPosition - initialPosition);
        double halfDistance = Math.abs(targetPosition - initialPosition) / 2.0;

        if(d1 > halfDistance) {
            t1 = Math.sqrt(2 * halfDistance / maxAcceleration);
            d1 = halfDistance;
        }
        double d2 = distance - 2 * d1;
        double t2 = d2 / maxVelocity;

        double t3 = t1;

        if(time < t1) {
            return (int) (0.5 * maxAcceleration * time * time + initialPosition);
        } else if(time < t1 + t2) {
            return (int) (0.5 * maxAcceleration * t1 * t1 + maxVelocity * (time - t1) + initialPosition);
        } else if(time < t1 + t2 + t3) {
            return (int) (0.5 * maxAcceleration * t1 * t1 + maxVelocity * t2 + maxVelocity * (time - t1 - t2) - 0.5 * maxAcceleration * (time - t1 - t2) * (time - t1 - t2) + initialPosition);
        } else {
            return targetPosition;
        }
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
}
