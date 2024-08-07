package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

// TODO: write class and builder class
public class BluServo implements BluHardwareDevice {
    String name;
    double pos=0, lastPos=0;

    ServoImplEx servo;

    public void setPos(double position) {
        pos = Range.clip(position, 0, 1);
    }

    public void init() {

    }

    public void read() {

    }

    public void write() {
        if(Math.abs(pos - lastPos) > 0.005) {
            servo.setPosition(pos);
            lastPos = pos;
        }
    }

    public void telemetry() {
        // telemetry.addLine(name + " pos: " + pos);
    }
}
