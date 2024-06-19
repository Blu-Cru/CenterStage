package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Dropdown implements Subsystem {
    public static double
            VERTICAL_POS = 0.5,
            P1toP2 = 40,
            P2toP3 = 96,
            P3axial = 48,
            P3tangential = 12,
            P3toOrigin = Math.hypot(P3axial, P3tangential), // origin is the axle where the dropdown rotates about
            THETA3 = Math.atan(P3tangential/P3axial);

    Servo wrist;
    private double position;

    public Dropdown(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "intake wrist");
    }

    public void init() {
        wrist.setPosition(position);
    }

    public void read() {

    }

    public void write() {
        if(wrist.getPosition() != position) {
            wrist.setPosition(position);
        }
    }

    public double toTicks(double angle) {
        double rawTicks = ((angle - Math.PI/2) / Math.toRadians(270)) + VERTICAL_POS;
        return Range.clip(rawTicks, 0.0, 1.0);
    }

    public static double toDeg(double height) {
        return -1;
    }

    public static double toX(int stackHeight) {
        return -1;
    }

    public void dropToStack(int stackHeight) {
    }

    public static double getTargetHeight(int stackHeight) {
        return -1;
    }

    private Point2d getP3(double targetAngle) {
        return Point2d.polar(P3toOrigin, Math.PI - THETA3 - targetAngle);
    }

    public void dropToGround() {}

    public void dropToPurpleHeight() {}

    public void retract() {}

    public void dropToAutoMidPos() {}

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake wrist pos", position);
    }
}
