package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

import java.util.HashMap;

public class IntakeColorSensors implements Subsystem {
    private enum SensorLocation {
        FRONT,
        BACK
    }

    private enum SlotState {
        EMPTY,
        FULL,
        WHITE,
        YELLOW,
        PURPLE,
        GREEN
    }

    static double[]
            FRONT_DISTANCE_RANGE = {0.05, 0.65}, // inches
            BACK_DISTANCE_RANGE = {0.1, 0.8}; // inches

    static double HUE_GREEN_HIGH_YELLOW_LOW = 270,
        HUE_PURPLE_HIGH_GREEN_LOW = 150,
        SATURATION_WHITE_HIGH = 0.3;

    HashMap<SensorLocation, double[]> ranges;

    SlotState frontSlotState, backSlotState;
    RevColorSensorV3 frontSensor, backSensor;
    NormalizedRGBA frontRGBA, backRGBA;
    double frontR, backR;
    double frontG, backG;
    double frontB, backB;
    float[] frontHSV, backHSV;
    double frontHue, backHue;
    double frontDistance, backDistance;
    double frontLightDetected, backLightDetected;
    double timeLastEmpty;
    boolean reading;

    public IntakeColorSensors(HardwareMap hardwareMap) {
        frontSensor = hardwareMap.get(RevColorSensorV3.class, "front color");
        backSensor = hardwareMap.get(RevColorSensorV3.class, "back color");

        ranges= new HashMap<SensorLocation, double[]>() {{
            put(SensorLocation.FRONT, FRONT_DISTANCE_RANGE);
            put(SensorLocation.BACK, BACK_DISTANCE_RANGE);
        }};

        frontSlotState = SlotState.EMPTY;
        backSlotState = SlotState.EMPTY;

        frontHSV = new float[3];
        backHSV = new float[3];

        reading = false;
    }

    public void init() {
        frontRGBA = frontSensor.getNormalizedColors();
        backRGBA = backSensor.getNormalizedColors();

        frontDistance = frontSensor.getDistance(DistanceUnit.INCH);
        backDistance = backSensor.getDistance(DistanceUnit.INCH);
        frontLightDetected = frontSensor.getLightDetected();
        backLightDetected = backSensor.getLightDetected();
    }

    public void read() {
        if(reading) {
            frontDistance = frontSensor.getDistance(DistanceUnit.INCH);
            backDistance = backSensor.getDistance(DistanceUnit.INCH);

            frontRGBA = frontSensor.getNormalizedColors();
            backRGBA = backSensor.getNormalizedColors();

            frontR = frontRGBA.red;
            frontG = frontRGBA.green;
            frontB = frontRGBA.blue;
            frontHSV = getHSV(frontRGBA); // sets frontHSV
            frontHue = frontHSV[0];

            backR = backRGBA.red;
            backG = backRGBA.green;
            backB = backRGBA.blue;
            backHSV = getHSV(backRGBA); // sets backHSV
            backHue = backHSV[0];

            frontSlotState = getSlotState(frontDistance, SensorLocation.FRONT, frontHSV);
            backSlotState = getSlotState(backDistance, SensorLocation.BACK, backHSV);
        }
    }

    public void write() {

    }

    public SlotState getSlotState(double distance, SensorLocation sensorLocation, float[] hsv) {
        if(!inRange(distance, ranges.get(sensorLocation))) {
            return SlotState.EMPTY;
        } else {
            if(hsv[2] > 0.022) {
                return SlotState.WHITE;
            } else if(hsv[0] <= 115) {
                return SlotState.YELLOW;
            } else if(hsv[0] > 155) {
                return SlotState.PURPLE;
            } else {
                return SlotState.GREEN;
            }
        }
    }

    // scales the front RGB values to be between 0 and 255
    public void scaleFrontRGB() {
        double max = Math.max(Math.max(frontR, frontB), frontG);
        frontR = frontR / max;
        frontG = frontG / max;
        frontB = frontB / max;
    }

    // scales the back RGB values to be between 0 and 255
    public void scaleBackRGB() {
        double max = Math.max(Math.max(backR, backB), backG);
        backR = backR / max;
        backG = backG / max;
        backB = backB / max;
    }

    public float[] getHSV(NormalizedRGBA color) {
        float[] hsv = new float[3];
        int colorInt = Color.argb((int)(color.alpha * 255), (int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
        Color.colorToHSV(colorInt, hsv);
//        Color.RGBToHSV((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), hsv);
        return hsv;
    }

    public void startReading() {
        reading = true;
    }

    public void stopReading() {
        reading = false;
    }

    public boolean isReading() {
        return reading;
    }

    public boolean isFull() {
        return frontSlotState != SlotState.EMPTY && backSlotState != SlotState.EMPTY;
    }

    private boolean inRange(double value, double[] range) {
        return value >= range[0] && value <= range[1];
    }

    public void reset() {
        frontSlotState = SlotState.EMPTY;
        backSlotState = SlotState.EMPTY;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FRONT SLOT STATE: ", frontSlotState);
        telemetry.addData("BACK SLOT STATE: ", backSlotState);
        telemetry.addData("front distance", frontDistance);
        telemetry.addData("back distance", backDistance);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("front R", frontR);
        telemetry.addData("front G", frontG);
        telemetry.addData("front B", frontB);
        telemetry.addData("front H:: ", frontHSV[0]);
        telemetry.addData("front S:: ", frontHSV[1]);
        telemetry.addData("front V:: ", frontHSV[2]);
        telemetry.addData("back R", backR);
        telemetry.addData("back G", backG);
        telemetry.addData("back B", backB);
        telemetry.addData("back H: ", backHSV[0]);
        telemetry.addData("back S: ", backHSV[1]);
        telemetry.addData("back V: ", backHSV[2]);
//        telemetry.addData("front light detected", frontLightDetected);
//        telemetry.addData("back light detected", backLightDetected);
    }
}
