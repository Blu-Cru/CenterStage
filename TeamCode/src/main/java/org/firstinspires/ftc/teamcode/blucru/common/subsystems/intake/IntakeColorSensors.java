package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.common.states.SlotState;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

public class IntakeColorSensors implements Subsystem {
    public enum SensorLocation {
        FRONT,
        BACK
    }

    public static double
            FRONT_DISTANCE_LOW = 0.05,
            FRONT_DISTANCE_HIGH = 0.65, // inches

            BACK_DISTANCE_LOW = 0.1,
            BACK_DISTANCE_HIGH = 0.5; // inches

    public static double BLUE_LOW_H = 80;
    public static double BLUE_HIGH_H = 140;
    public static double PURPLE_LOW_H = 250;
    public static double PURPLE_HIGH_H = 300;
    public static double YELLOW_LOW_H = 40;
    public static double YELLOW_HIGH_H = 60;
    public static double GREEN_LOW_H = 100;
    public static double GREEN_HIGH_H = 140;

    public SlotState frontSlotState, backSlotState;
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
//        if(reading) {
////            frontSensor.enableLed(true);
////            backSensor.enableLed(true);
////            frontRGBA = frontSensor.getNormalizedColors();
////            backRGBA = backSensor.getNormalizedColors();
//
////            frontR = frontRGBA.red;
////            frontG = frontRGBA.green;
////            frontB = frontRGBA.blue;
////            getHSV(frontRGBA, frontHSV); // sets frontHSV
////            frontHue = frontHSV[0];
////
////            backR = backRGBA.red;
////            backG = backRGBA.green;
////            backB = backRGBA.blue;
////            getHSV(backRGBA, backHSV); // sets backHSV
////            backHue = backHSV[0];
//
//            frontDistance = frontSensor.getDistance(DistanceUnit.INCH);
//            backDistance = backSensor.getDistance(DistanceUnit.INCH);
////            frontLightDetected = frontSensor.getLightDetected();
////            backLightDetected = backSensor.getLightDetected();
//
//            frontSlotState = calculateSlotState(frontDistance, SensorLocation.FRONT);
//            backSlotState = calculateSlotState(backDistance, SensorLocation.BACK);
//        } else {
//            frontSensor.enableLed(false);
//            backSensor.enableLed(false);
//        }
        frontSlotState = SlotState.EMPTY;
        backSlotState = SlotState.EMPTY;
    }

    public void write() {

    }

    public SlotState calculateSlotState(double distance, SensorLocation sensor) {
        if(sensor == SensorLocation.FRONT) {
            if(distance >= FRONT_DISTANCE_LOW && distance <= FRONT_DISTANCE_HIGH)
                return SlotState.FULL;
            else
                return SlotState.EMPTY;
        } else {
            if(distance >= BACK_DISTANCE_LOW && distance <= BACK_DISTANCE_HIGH)
                return SlotState.FULL;
            else
                return SlotState.EMPTY;
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

//    public double getHue(NormalizedRGBA color) {
//        return getHue(color.red, color.green, color.blue);
//    }

    public void getHSV(NormalizedRGBA color, float[] hsv) {
        Color.RGBToHSV((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), hsv);
    }

    // calculates hue in degrees (0-360)
    public double getHue(double r, double g, double b) {
        double max = Math.max(Math.max(r, b), g);
        double min = Math.min(Math.min(r, b), g);
        double delta = max - min;
        if(delta == 0) return 0;
        if(max == r) return 60 * (((g - b) / delta) % 6);
        if(max == g) return 60 * (((b - r) / delta) + 2);
        if(max == b) return 60 * (((r - g) / delta) + 4);
        return 0;
    }

    public double getSaturation(double r, double g, double b) {
        double max = Math.max(Math.max(r, b), g);
        double min = Math.min(Math.min(r, b), g);
        double delta = max - min;
        if(max == 0) return 0;
        return delta / max;
    }

    public float[] getFrontHSV() {
        return frontHSV;
    }

    public float[] getBackHSV() {
        return backHSV;
    }

    public void startReading() {
        reading = true;
    }

    public void stopReading() {
        reading = false;
    }

    public boolean isFull() {
        return frontSlotState == SlotState.FULL && backSlotState == SlotState.FULL;
    }

    public void toggleReading() {
        reading = !reading;
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
        telemetry.addData("front hue", frontHue);
        telemetry.addData("back R", backR);
        telemetry.addData("back G", backG);
        telemetry.addData("back B", backB);
        telemetry.addData("back hue", backHue);
        telemetry.addData("front light detected", frontLightDetected);
        telemetry.addData("back light detected", backLightDetected);
    }
}
