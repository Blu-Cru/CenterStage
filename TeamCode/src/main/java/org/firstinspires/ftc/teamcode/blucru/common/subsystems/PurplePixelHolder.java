package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PurplePixelHolder implements Subsystem{
    public static double EXTENDED = 0.49;
    public static double RETRACTED_RIGHT = EXTENDED - 0.333;
    public static double RETRACTED_LEFT = EXTENDED + 0.333;

    enum PurplePixelState {
        RETRACTED_RIGHT,
        RETRACTED_LEFT,
        EXTENDED
    }

    private Servo purplePixelHolder;

    PurplePixelState purplePixelState;
    private double position;

    public PurplePixelHolder(HardwareMap hardwareMap) {
        purplePixelHolder = hardwareMap.get(Servo.class, "purple pixel");
        purplePixelState = PurplePixelState.EXTENDED;
    }

    public void init() {
        purplePixelHolder.setPosition(EXTENDED);
    }

    public void read() {
        switch(purplePixelState) {
            case RETRACTED_RIGHT:
                position = RETRACTED_RIGHT;
                break;
            case RETRACTED_LEFT:
                position = RETRACTED_LEFT;
                break;
            case EXTENDED:
                position = EXTENDED;
                break;
        }
    }

    public void write() {
        if(purplePixelHolder.getPosition() != position) purplePixelHolder.setPosition(position);
    }

    public void retractRight() {
        purplePixelState = PurplePixelState.RETRACTED_RIGHT;
    }

    public void retractLeft() {
        purplePixelState = PurplePixelState.RETRACTED_LEFT;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Purple Pixel state", purplePixelState);
    }
}
