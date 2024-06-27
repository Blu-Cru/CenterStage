package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Turret implements Subsystem {
    enum State {
        IDLE,
        IVK
    }

    public static final double
            BUCKET_LENGTH = 5.984,
            BUCKET_WIDTH = 1.733, // inches
            BUCKET_HYPOTENUSE = Math.sqrt(BUCKET_LENGTH * BUCKET_LENGTH + BUCKET_WIDTH * BUCKET_WIDTH);
    public static double TURRET_CENTER = 0.58, // ticks of turret servo at 270 degrees, pointing straight down/forward
            MAX_TURRET_X = 5.3; // inches

    Servo turretServo;

    private double targetAngle;
    private double position;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turret");
        turretServo.setDirection(Servo.Direction.REVERSE);
        targetAngle = 270;
        position = toTicks(targetAngle);
    }

    public void init() {
        turretServo.setPosition(position);
    }

    public void read() {

    }

    public void write() {
        position = toTicks(targetAngle);
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

    public void setX(double x) {
        x = Range.clip(x, -MAX_TURRET_X, MAX_TURRET_X);
        targetAngle = xToAngle(x);
    }

    public void setAngle(double angle) {
        targetAngle = angle;
    }

    public double getAngle() {
        return targetAngle;
    }

    public void setGlobalY(double globalY) {
        double dtY = Robot.getInstance().drivetrain.pose.getY();

        setX(dtY - globalY);
    }

    public boolean isCentered() {
        return Math.abs(targetAngle - 270) < 1;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("turret angle", targetAngle);
    }

    public static double xToAngle(double x) { // positive is counterclockwise in deposit position
        return Math.toDegrees(Math.asin(x / BUCKET_LENGTH)) + 270;
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("turret position", position);
    }
}
