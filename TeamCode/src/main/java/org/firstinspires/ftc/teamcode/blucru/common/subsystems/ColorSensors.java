package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensors implements Subsystem{
    RevColorSensorV3 frontColor, backColor;
    NormalizedRGBA frontRGBA, backRGBA;
    boolean reading;

    public ColorSensors(HardwareMap hardwareMap) {
        frontColor = hardwareMap.get(RevColorSensorV3.class, "front color");
        backColor = hardwareMap.get(RevColorSensorV3.class, "back color");
        frontRGBA = frontColor.getNormalizedColors();
        backRGBA = backColor.getNormalizedColors();
        reading = false;
    }

    public void init() {
        reading = false;
    }

    public void read() {
        if(reading) {
            frontRGBA = frontColor.getNormalizedColors();
            backRGBA = backColor.getNormalizedColors();
        }
    }

    public void write() {

    }

    public void telemetry(Telemetry telemetry) {

    }
}
