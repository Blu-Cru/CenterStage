package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Plane implements Subsystem{
    public static double PLANE_RELEASED = 0.5;
    public static double PLANE_RETRACT = 0.5;

    Servo plane;
    ServoControllerEx planeController;

    boolean released = false;

    public Plane(HardwareMap hardwareMap) {
        plane = hardwareMap.get(Servo.class, "plane");
    }

    @Override
    public void init() {
        plane.setPosition(PLANE_RETRACT);
    }

    public void read() {

    }

    @Override
    public void write() {
        if(released) {
            plane.setPosition(PLANE_RELEASED);
        } else {
            plane.setPosition(PLANE_RETRACT);
        }
    }

    public void togglePlane() {
        released = !released;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("plane released", released);
    }
}
