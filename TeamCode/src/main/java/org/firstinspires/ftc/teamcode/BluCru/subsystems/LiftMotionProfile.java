package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftMotionProfile {
    public static double fastVelocity = 10000;
    public static double fastAccel = 10000;

    public double vMax;
    public double aMax;
    public int xTarget;
    public int xI;
    public double vI;
    public double flip;
    double t0, t1, t2, t3;
    double d0, d1, d2, d3;

    public LiftMotionProfile(int xTarget, int xI) {
        this.xTarget = xTarget;
        this.xI = xI;
        this.vMax = 1000;
        this.vI = 0;
        this.aMax = 2000;
        if(xTarget < xI) {
            flip = -1;
        } else {
            flip = 1;
        }
    }

    public LiftMotionProfile(int xTarget, int xI, double vMax, double aMax) {
        this.xTarget = xTarget;
        this.xI = xI;
        this.vMax = vMax;
        this.vI = 0;
        this.aMax = aMax;
        if(xTarget < xI) {
            flip = -1;
        } else {
            flip = 1;
        }
    }

    public LiftMotionProfile(int xTarget, int xI, double vI, double vMax, double aMax) {
        this.xTarget = xTarget;
        this.xI = xI;
        this.vMax = vMax;
        this.vI = vI;
        this.aMax = aMax;
        if(xTarget < xI) {
            flip = -1;
        } else {
            flip = 1;
        }
    }

    public int calculateTargetPosition(double time) {
        t1 = (vMax - vI) / aMax;
        d1 = (0.5 * aMax * t1 * t1) + (vI * t1);

        // 0 is the period before initial, when the lift accelerates from 0 velo to vi
        t0 = vI / aMax;
        d0 = (vI * vI) / (2.0 * aMax);

        double distance = Math.abs(xTarget - xI) + d0;
        double halfDistance = distance / 2.0;

        if(d1 + d0 > halfDistance) {
            d1 = halfDistance - d0;
            t1 = Math.sqrt(2 * halfDistance / aMax) - t0;
        }

        vMax = aMax * t1 + vI;

        // 2 is cruise period, lift is at max velo
        d2 = distance - 2 * (d0 + d1);
        t2 = d2 / vMax;

        t3 = vMax / aMax;

        if(time < t1) {
            return (int) ((0.5 * aMax * time * time + vI * time) * flip + xI);
        } else if(time < t1 + t2) {
            return (int) ((d1 + vMax * (time - t1)) * flip + xI);
        } else if(time < t1 + t2 + t3) {
            return (int) ((d1 + d2 + vMax * (time - t1 - t2) - 0.5 * aMax * (time - t1 - t2) * (time - t1 - t2)) * flip + xI);
        } else {
            return xTarget;
        }
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.vMax = maxVelocity;
        this.aMax = maxAcceleration;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("t1", t1);
        telemetry.addData("t2", t2);
        telemetry.addData("t3", t3);
        telemetry.addData("d1", d1);
        telemetry.addData("d2", d2);
        telemetry.addData("d3", d3);
        telemetry.addData("target position", xTarget);
        telemetry.addData("initial position", xI);
        telemetry.addData("max velocity", vMax);
        telemetry.addData("max acceleration", aMax);
    }
}
