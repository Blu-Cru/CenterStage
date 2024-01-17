package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Plane implements Subsystem{
    public static double PLANE_RELEASED = 0.5;

    Servo plane;
    ServoControllerEx planeController;

    boolean released = false;

    public Plane(HardwareMap hardwareMap) {
        plane = hardwareMap.get(Servo.class, "plane");
        planeController = (ServoControllerEx) plane.getController();
    }

    @Override
    public void init() {
        planeController.pwmDisable();
    }

    public void read() {

    }

    @Override
    public void write() {
        if(released) {
            planeController.pwmEnable();
            plane.setPosition(PLANE_RELEASED);
        } else {
            planeController.pwmDisable();
        }
    }

    public void releasePlane() {
        released = true;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("plane released", released);
    }
}
