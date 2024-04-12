package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret implements Subsystem{
    public static final double
            BUCKET_LENGTH = 5.984,
            BUCKET_WIDTH = 1.733, // inches
            BUCKET_HYPOTENUSE = Math.sqrt(BUCKET_LENGTH * BUCKET_LENGTH + BUCKET_WIDTH * BUCKET_WIDTH);
    public static double TURRET_CENTER = 0.49; // ticks of turret servo at 270 degrees

    Servo turretServo;

    public double targetAngle;
    private double position;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turret");
        targetAngle = 270;
        position = toTicks(targetAngle);
    }

    public void init() {
        turretServo.setPosition(position);
    }

    public void read() {
        position = toTicks(targetAngle);
    }

    public void write() {
        if(turretServo.getPosition() != position) {
            turretServo.setPosition(position);
        }
    }

    // converts turret angle degrees to ticks
    public double toTicks(double targetAngle) {
        double rawTicks = ((targetAngle - 270.0) /270.0) + TURRET_CENTER;
        return Range.clip(rawTicks, 0.0, 1.0);
    }

    // returns the change in height due to the turret angle (turret pointing downward is negative)
    public double getTurretHeightDelta() {
        return BUCKET_LENGTH * Math.sin(Math.toRadians(targetAngle)) - Math.abs(BUCKET_WIDTH * Math.cos(Math.toRadians(targetAngle)));
    }

    public boolean isCentered() {
        return Math.abs(targetAngle - 270) < 2;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("turret angle", targetAngle);
    }

    public static double xToAngle(double x) {
        return Math.toDegrees(Math.asin(x / BUCKET_LENGTH)) + 270;
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("turret position", position);
    }
}
