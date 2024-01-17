package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotionProfile {
    public double vMax;
    public double aMax;
    public int xTarget;
    public int xI;
    public double vI;
    public double flip;
    public boolean decel;
    public double xDecel;
    double t0, t1, t2, t3;
    double d0, d1, d2, d3;
    double v0, v1, v2, v3;
    double a0, a1, a2, a3;

    public MotionProfile(int xTarget, int xI) {
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
        calculate();
    }

    public MotionProfile(int xTarget, int xI, double vMax, double aMax) {
        this.xTarget = xTarget;
        this.xI = xI;
        this.vMax = vMax;
        this.vI = 0;
        this.aMax = aMax;
        if(xTarget < xI) {
            flip = -1;
            decel = vI > 0;
        } else {
            flip = 1;
            decel = vI < 0;
        }
        calculate();
    }

    public MotionProfile(int xTarget, int xI, double vI, double vMax, double aMax) {
        this.xTarget = xTarget;
        this.xI = xI;
        this.vMax = vMax;
        this.vI = vI;
        this.aMax = aMax;
        if(xTarget < xI) {
            flip = -1;
//            decel = vI > 0;
        } else {
            flip = 1;
//            decel = vI < 0;
        }

        xDecel = vI < 0 ? -(vI * vI) / (2.0 * aMax) : (vI * vI) / (2.0 * aMax);

        if(vI < 0 && xTarget > xI + xDecel) {
            decel = true;
        } else if(vI > 0 && xTarget < xI + xDecel) {
            decel = true;
        } else {
            decel = false;
        }

        // absolute delta x to stop

        calculate();
    }

    public void calculate() {
        if(decel) {
            // time it takes to stop
            t0 = Math.abs(vI / aMax);
            // displacement for stopping (should be flipped)
            d0 = -(0.5 * aMax * t0 * t0);

            // time to accel to max velocity
            t1 = vMax / aMax;
            // distance to accel
            d1 = 0.5 * aMax * t1 * t1;

            double distance = Math.abs(xTarget - xI - xDecel);
            double halfDistance = distance / 2.0;

            if(d1 > halfDistance) {
                t1 = Math.sqrt(2 * halfDistance / aMax);
            }
            d1 = 0.5 * aMax * t1 * t1;
            vMax = aMax * t1;

            d2 = distance - 2 * d1;
            t2 = d2 / vMax;

            t3 = t1;
            d3 = d1;
        } else {
            t0 = 0.0;
            d0 = 0.0;

            t1 = Math.abs((vMax * flip - vI) / aMax);
            d1 = Math.abs((0.5 * aMax * t1 * t1) * flip + (vI * t1));

            double xAccel = (vI * vI) / (2.0 * aMax);
            double distance = Math.abs(xTarget - xI) + xAccel;
            double halfDistance = distance / 2.0;

            if(d1 > halfDistance - xAccel) {
                d1 = halfDistance - xAccel;
                t1 = Math.sqrt(2 * halfDistance / aMax);
            }

            vMax = aMax * t1 + Math.abs(vI);

            d2 = distance - 2 * (xAccel + d1);
            t2 = d2 / vMax;

            t3 = vMax / aMax;
        }
    }

    public int calculateTargetPosition(double time) {
        int instantTargetPos;
        double dt;

        if(time < t0) {
            dt = time;
            instantTargetPos = (int) ((vI * dt) + ((aMax * dt * dt / 2.0) * flip) + xI);
        } else if(time < t0 + t1) {
            dt = time - t0;
            instantTargetPos = (int) ((d0 + 0.5 * aMax * dt * dt) * flip + xI);
        } else if(time < t0 + t1 + t2) {
            dt = time - t0 - t1;
            instantTargetPos = (int) ((d0 + d1 + vMax * dt) * flip + xI);
        } else if(time < t0 + t1 + t2 + t3) {
            dt = time - t0 - t1 - t2;
            instantTargetPos = (int) ((d0 + d1 + d2 + vMax * dt - 0.5 * aMax * dt * dt) * flip + xI);
        } else {
            instantTargetPos = xTarget;
        }

        return instantTargetPos;
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.vMax = maxVelocity;
        this.aMax = maxAcceleration;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("t0", t0);
        telemetry.addData("t1", t1);
        telemetry.addData("t2", t2);
        telemetry.addData("t3", t3);
        telemetry.addData("d0", d0);
        telemetry.addData("d1", d1);
        telemetry.addData("d2", d2);
        telemetry.addData("d3", d3);
        telemetry.addData("vI", vI);
        telemetry.addData("target position", xTarget);
        telemetry.addData("initial position", xI);
        telemetry.addData("max velocity", vMax);
        telemetry.addData("max acceleration", aMax);
    }
}
