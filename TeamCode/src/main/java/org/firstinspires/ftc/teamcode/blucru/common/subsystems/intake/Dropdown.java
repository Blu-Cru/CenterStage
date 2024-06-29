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
            VERTICAL_POS = 0.24,
            L1 = 40,
            L2 = 96,
            P3axial = 48,
            P3tangential = 12,
            DROPDOWN_LENGTH = 144,
            P3toOrigin = Math.hypot(P3axial, P3tangential), // origin is the axle where the dropdown rotates about
            THETA3 = Math.atan(P3tangential/P3axial),

            PIXEL_HEIGHT = 12.7,
            STACK_1_HEIGHT = 40,
            RETRACT_HEIGHT = 130,

            AUTO_MID_HEIGHT = 110,

            GROUND_HEIGHT = 27;

    static Point2d P1 = new Point2d(79.95, 50);

    Servo wrist;
    private double position;

    public Dropdown(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "dropdown");
        wrist.setDirection(Servo.Direction.REVERSE);
        position = VERTICAL_POS;
    }

    public Dropdown() {

    }

    public void init() {
        setTargetHeight(DROPDOWN_LENGTH);
        wrist.setPosition(position);
    }

    public void read() {

    }

    public void write() {
        if(wrist.getPosition() != position) {
            wrist.setPosition(position);
        }
    }

    public void setTargetHeight(double targetHeight) {
        position = toTicks(getServoAngle(getP3(getDropdownAngle(targetHeight))));
    }

    public double getDropdownAngle(double targetHeight) {
        return Math.asin(targetHeight / DROPDOWN_LENGTH);
    }

    public Point2d getP3(double dropdownAngle) {
        return Point2d.polar(P3toOrigin, Math.PI - THETA3 - dropdownAngle);
    }

    public double getServoAngle(Point2d p3) {
        double distance = p3.distance(P1);
        double interiorAngle = Math.acos((-L2 * L2 + distance * distance + L1 * L1)/(2 * distance * L1));
        double deltaAngle = Math.atan((P1.y - p3.y)/(P1.x - p3.x));
        return Math.PI + deltaAngle - interiorAngle;
    }

    public void dropToStack(int stackHeight) {
        setTargetHeight(getTargetHeight(stackHeight));
    }

    private double getTargetHeight(int stackHeight) {
        if(stackHeight <= 0) {
            return GROUND_HEIGHT;
        }
        else {
            stackHeight = Range.clip(stackHeight, 1, 5);
            return STACK_1_HEIGHT + PIXEL_HEIGHT * (stackHeight-1);
        }
    }

    public void dropToGround() {
        dropToStack(0);
    }

    public void retract() {
        setTargetHeight(RETRACT_HEIGHT);
    }

    public void dropToAutoMidPos() {
        setTargetHeight(AUTO_MID_HEIGHT);
    }

    private double toTicks(double angle) {
        double rawTicks = ((angle - Math.PI/2) / Math.toRadians(270)) + VERTICAL_POS;
        return Range.clip(rawTicks, 0.0, 1.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake wrist pos", position);
    }
}
