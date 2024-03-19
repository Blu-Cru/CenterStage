package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeColorSensors implements Subsystem{
    public static double BLUE_LOW_H = 80;
    public static double BLUE_HIGH_H = 140;
    public static double PURPLE_LOW_H = 250;
    public static double PURPLE_HIGH_H = 300;
    public static double YELLOW_LOW_H = 40;
    public static double YELLOW_HIGH_H = 60;
    public static double GREEN_LOW_H = 100;
    public static double GREEN_HIGH_H = 140;

    RevColorSensorV3 frontSensor, backSensor;
    NormalizedRGBA frontRGBA, backRGBA;
    double frontR, backR;
    double frontG, backG;
    double frontB, backB;
    float[] frontHSV, backHSV;
    double frontHue, backHue;
    double frontDistance, backDistance;
    double frontLightDetected, backLightDetected;
    boolean reading;

    public IntakeColorSensors(HardwareMap hardwareMap) {
        frontSensor = hardwareMap.get(RevColorSensorV3.class, "front color");
        backSensor = hardwareMap.get(RevColorSensorV3.class, "back color");

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
            frontSensor.enableLed(true);
            backSensor.enableLed(true);
            frontRGBA = frontSensor.getNormalizedColors();
            backRGBA = backSensor.getNormalizedColors();

            frontR = frontRGBA.red;
            frontG = frontRGBA.green;
            frontB = frontRGBA.blue;
            getHSV(frontRGBA, frontHSV);
            frontHue = frontHSV[0];

            backR = backRGBA.red;
            backG = backRGBA.green;
            backB = backRGBA.blue;
            getHSV(backRGBA, backHSV);
            backHue = backHSV[0];

            frontDistance = frontSensor.getDistance(DistanceUnit.INCH);
            backDistance = backSensor.getDistance(DistanceUnit.INCH);
            frontLightDetected = frontSensor.getLightDetected();
            backLightDetected = backSensor.getLightDetected();
        } else {
            frontSensor.enableLed(false);
            backSensor.enableLed(false);
        }
    }

    public void write() {

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

    public void toggleReading() {
        reading = !reading;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("READING: ", reading);
        telemetry.addData("front R", frontR);
        telemetry.addData("front G", frontG);
        telemetry.addData("front B", frontB);
        telemetry.addData("front hue", frontHue);
        telemetry.addData("back R", backR);
        telemetry.addData("back G", backG);
        telemetry.addData("back B", backB);
        telemetry.addData("back hue", backHue);
        telemetry.addData("front distance", frontDistance);
        telemetry.addData("back distance", backDistance);
        telemetry.addData("front light detected", frontLightDetected);
        telemetry.addData("back light detected", backLightDetected);
    }
}
