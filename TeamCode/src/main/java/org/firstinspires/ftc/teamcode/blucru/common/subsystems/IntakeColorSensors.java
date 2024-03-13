package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeColorSensors implements Subsystem{
    RevColorSensorV3 frontSensor, backSensor;
    NormalizedRGBA frontRGBA, backRGBA;
    double frontDistance, backDistance;
    double frontLightDetected, backLightDetected;
    boolean reading;

    public IntakeColorSensors(HardwareMap hardwareMap) {
        frontSensor = hardwareMap.get(RevColorSensorV3.class, "front color");
//        backSensor = hardwareMap.get(RevColorSensorV3.class, "back color");

        reading = false;
    }

    public void init() {
        frontRGBA = frontSensor.getNormalizedColors();
//        backRGBA = backSensor.getNormalizedColors();
        frontDistance = frontSensor.getDistance(DistanceUnit.INCH);
//        backDistance = backSensor.getDistance(DistanceUnit.INCH);
        frontLightDetected = frontSensor.getLightDetected();
//        backLightDetected = backSensor.getLightDetected();
    }

    public void read() {
        if(reading) {
            frontRGBA = frontSensor.getNormalizedColors();
//            backRGBA = backSensor.getNormalizedColors();
            frontDistance = frontSensor.getDistance(DistanceUnit.INCH);
//            backDistance = backSensor.getDistance(DistanceUnit.INCH);
            frontLightDetected = frontSensor.getLightDetected();
//            backLightDetected = backSensor.getLightDetected();
        }
    }

    public void write() {

    }

    public void startReading() {
        reading = true;
    }

    public void stopReading() {
        reading = false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("READING: ", reading);
        telemetry.addData("front color", frontRGBA.toString());
//        telemetry.addData("back color", backRGBA.toString());
        telemetry.addData("front distance", frontDistance);
//        telemetry.addData("back distance", backDistance);
        telemetry.addData("front light detected", frontLightDetected);
//        telemetry.addData("back light detected", backLightDetected);
    }
}
