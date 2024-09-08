package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

// prob no need for builder
public class BluServo extends ServoImpl implements BluHardwareDevice {
    String name;
    ServoController controller;
    double pos=0, lastPos=0;

    public BluServo(String name, Direction direction) {
        super(Globals.hwMap.get(ServoImpl.class, name).getController(),
                Globals.hwMap.get(ServoImpl.class, name).getPortNumber(), direction);
        this.name = name;
        controller = getController();
    }

    public BluServo(String name) {
        this(name, Direction.FORWARD);
    }

    public void setPosition(double position) {
        pos = Range.clip(position, 0, 1);
    }

    public void init() {
    }

    public void read() {

    }

    public void write() {
        if(Math.abs(pos - lastPos) > 0.005) {
            super.setPosition(pos);
            lastPos = pos;
        }
    }

    public void telemetry() {
         Globals.tele.addLine(name + " pos: " + pos);
    }
}
