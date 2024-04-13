package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Plane implements Subsystem {
    public static double
            PLANE_RELEASED = 0.65,
            PLANE_RETRACT = 0.44;

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
