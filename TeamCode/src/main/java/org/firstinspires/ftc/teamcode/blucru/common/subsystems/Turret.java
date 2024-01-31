package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret implements Subsystem{
    public final double TURRET_RADIUS = 5.984; // inches
    public final double BUCKET_WIDTH = 1.733; // inches
    public static double TURRET_CENTER = 0.52; // position of turret servo at 270 degrees

    private Servo turret;

    public double targetAngle;
    private double position;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "turret");
        targetAngle = 270;
        position = toTicks(targetAngle);
    }

    public void init() {
        turret.setPosition(position);
    }

    public void read() {
        position = toTicks(targetAngle);
    }

    public void write() {
        if(turret.getPosition() != position) {
            turret.setPosition(position);
        }
    }

    public double toTicks(double targetAngle) {
        double rawTicks = ((targetAngle - 270.0) /270.0) + TURRET_CENTER;
        return Range.clip(rawTicks, 0.0, 1.0);
    }

    public double getTurretHeightDelta() {
        return TURRET_RADIUS * Math.sin(Math.toRadians(targetAngle)) - Math.abs(BUCKET_WIDTH * Math.cos(Math.toRadians(targetAngle)));
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("turret angle", targetAngle);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("turret position", position);
    }
}
