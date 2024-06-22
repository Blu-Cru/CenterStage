package org.firstinspires.ftc.teamcode.blucru.common.subsystems.purple;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

public class PurplePixelHolder implements Subsystem {
    public static double
            EXTENDED = 0.49,
            RETRACTED_RIGHT = EXTENDED - 0.333,
            RETRACTED_LEFT = EXTENDED + 0.333;

    enum PurplePixelState {
        RETRACTED_RIGHT,
        RETRACTED_LEFT,
        EXTENDED
    }

    Servo purplePixelServo;

    PurplePixelState purplePixelState;
    private double position;

    public PurplePixelHolder(HardwareMap hardwareMap) {
        purplePixelServo = hardwareMap.get(Servo.class, "purple");
        purplePixelState = PurplePixelState.EXTENDED;
    }

    public void init() {
        purplePixelServo.setPosition(EXTENDED);
    }

    public void read() {

    }

    public void write() {
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

        if(purplePixelServo.getPosition() != position) purplePixelServo.setPosition(position);
    }

    public void retractRight() {
        purplePixelState = PurplePixelState.RETRACTED_RIGHT;
    }

    public void release(double reflect) {
        purplePixelState = reflect > 0 ? PurplePixelState.RETRACTED_LEFT : PurplePixelState.RETRACTED_RIGHT;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Purple Pixel state", purplePixelState);
    }
}
