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
            VERTICAL_POS = 0.24, // servo position where the first bar is vertical
            L1 = 40, // length of first bar
            L2 = 98, // length of 2nd bar (was zip tie)
            P3axial = 48, P3tangential = 12, // point where zip tie connected to dropdown with respect to the dropdown
            DROPDOWN_LENGTH = 96, // length from center of rotation axle to center of flipper axle
            P3toOrigin = Math.hypot(P3axial, P3tangential), // origin is the axle where the dropdown rotates about
            THETA3 = Math.atan(P3tangential/P3axial), // angle of P3 with respect to dropdown angle

            PIXEL_HEIGHT = 12.7, // height of each pixel in mm
            STACK_1_HEIGHT = 38, // height in mm where 1 pixel fits under dropdown
            RETRACT_HEIGHT = 90,

            AUTO_MID_HEIGHT = 110,

            GROUND_HEIGHT = 15; // height in mm to drop to the ground

    static Point2d P1 = new Point2d(79.95, 50);

    double[] positionsForHeights = new double[6];

    Servo pivotServo;
    private double position;

    public Dropdown(HardwareMap hardwareMap) {
        pivotServo = hardwareMap.get(Servo.class, "dropdown");
        pivotServo.setDirection(Servo.Direction.REVERSE);
        position = VERTICAL_POS;
    }

    // default constructor for unit testing
    public Dropdown() {}

    public void init() {
        setTargetHeight(DROPDOWN_LENGTH);
        pivotServo.setPosition(position);
//        calculatePositionsForHeights(); // TODO: for next time, cache positions instead of calculating every time
    }

    public void read() {

    }

    public void write() {
        if(pivotServo.getPosition() != position) {
            pivotServo.setPosition(position);
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

    // this method should be used to cache the positions for each height instead of calculating in real time, but i was idiot and didn't use it
    public void calculatePositionsForHeights() {
        for(int i = 0; i < 6; i++) {
            positionsForHeights[i] = toTicks(getServoAngle(getP3(getDropdownAngle(getTargetHeight(i)))));
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake wrist pos", position);
    }
}
