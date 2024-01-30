package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Plane implements Subsystem{
    public static double PLANE_RELEASED = 0.6;
    public static double PLANE_RETRACT = 0.42;

    Servo plane;

    boolean released = false;

    double position;

    public Plane(HardwareMap hardwareMap) {
        plane = hardwareMap.get(Servo.class, "plane");
    }

    @Override
    public void init() {
        plane.setPosition(PLANE_RETRACT);
    }

    public void read() {
        position = released ? PLANE_RELEASED : PLANE_RETRACT;
    }

    @Override
    public void write() {
        if(plane.getPosition() != position) {
            plane.setPosition(position);
        }
    }

    public void togglePlane() {
        released = !released;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("plane released", released);
    }
}
