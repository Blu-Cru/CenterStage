package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PurplePixelHolder implements Subsystem{
    public static double EXTENDED = 0.49;
    public static double RETRACTED = EXTENDED - 0.333;

    private Servo purplePixelHolder;

    public boolean retracted = false;
    private double position;

    public PurplePixelHolder(HardwareMap hardwareMap) {
        purplePixelHolder = hardwareMap.get(Servo.class, "purple pixel");
    }

    public void init() {
        purplePixelHolder.setPosition(EXTENDED);
    }

    public void read() {
        position = retracted ? RETRACTED : EXTENDED;
    }

    public void write() {
        if(purplePixelHolder.getPosition() != position) purplePixelHolder.setPosition(position);
    }

    public void retract() {
        retracted = true;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("purple pixel retracted", retracted);
    }
}
