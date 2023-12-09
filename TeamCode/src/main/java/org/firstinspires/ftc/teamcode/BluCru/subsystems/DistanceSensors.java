package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DistanceSensors implements Subsystem {
    DistanceSensor rightDistanceSensor;
    DistanceSensor leftDistanceSensor;

    public DistanceSensors(HardwareMap hardwareMap) {
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance");
    }
    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
