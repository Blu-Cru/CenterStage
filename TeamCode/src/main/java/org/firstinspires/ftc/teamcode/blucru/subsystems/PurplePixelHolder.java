package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PurplePixelHolder implements Subsystem{
    public static double EXTENDED = 0.5;
    public static double RETRACTED = EXTENDED - 0.333;

    private Servo purplePixelHolder;

    boolean retracted = true;
    private double position;

    public PurplePixelHolder(HardwareMap hardwareMap) {
        purplePixelHolder = hardwareMap.get(Servo.class, "purplePixelHolder");
    }

    public void init() {
        purplePixelHolder.setPosition(RETRACTED);
    }

    public void read() {
        position = retracted ? RETRACTED : EXTENDED;
    }

    public void write() {
        purplePixelHolder.setPosition(position);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("purple pixel retracted", retracted);
    }
}
