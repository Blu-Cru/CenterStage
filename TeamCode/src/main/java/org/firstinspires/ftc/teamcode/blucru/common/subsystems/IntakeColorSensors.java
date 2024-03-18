package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeColorSensors implements Subsystem{
    RevColorSensorV3 frontSensor, backSensor;
    NormalizedRGBA frontRGBA, backRGBA;
    double frontR, frontG, frontB;
    double backR, backG, backB;
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
            scaleFrontRGB();

            backR = backRGBA.red;
            backG = backRGBA.green;
            backB = backRGBA.blue;
            scaleBackRGB();

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

    public void scaleFrontRGB() {
        double max = Math.max(Math.max(frontR, frontB), frontG);
        frontR = frontR * 255.0 / max;
        frontG = frontG * 255.0 / max;
        frontB = frontB * 255.0 / max;
    }

    public void scaleBackRGB() {
        double max = Math.max(Math.max(backR, backB), backG);
        backR = backR * 255.0 / max;
        backG = backG * 255.0 / max;
        backB = backB * 255.0 / max;
    }

    public void startReading() {
        reading = true;
    }

    public void stopReading() {
        reading = false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("READING: ", reading);
        telemetry.addData("front R", frontR);
        telemetry.addData("front G", frontG);
        telemetry.addData("front B", frontB);
        telemetry.addData("front A", frontRGBA.alpha);
        telemetry.addData("back R", backR);
        telemetry.addData("back G", backG);
        telemetry.addData("back B", backB);
        telemetry.addData("back A", backRGBA.alpha);
        telemetry.addData("front distance", frontDistance);
        telemetry.addData("back distance", backDistance);
        telemetry.addData("front light detected", frontLightDetected);
        telemetry.addData("back light detected", backLightDetected);
    }
}
