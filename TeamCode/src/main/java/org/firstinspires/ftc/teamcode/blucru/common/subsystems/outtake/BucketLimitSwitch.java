package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

public class BucketLimitSwitch implements Subsystem {
    DigitalChannel limitSwitch;
    boolean reading;
    boolean isPressed;

    public BucketLimitSwitch(HardwareMap hardwareMap) {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "bucket switch");
        isPressed = false;
        reading = false;
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {
        if(reading) {
            isPressed = !limitSwitch.getState();
        }
    }

    @Override
    public void write() {

    }

    public boolean isPressed() {
        return isPressed;
    }

    public void startReading() {
        reading = true;
    }

    public void stopReading() {
        reading = false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Limit switch state:", isPressed);
    }
}
