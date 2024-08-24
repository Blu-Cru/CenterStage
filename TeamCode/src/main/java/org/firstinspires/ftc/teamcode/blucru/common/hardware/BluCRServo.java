package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class BluCRServo extends CRServoImpl implements BluHardwareDevice{
    String name;
    double power, lastPower;

    public BluCRServo(String name, Direction direction) {
        super(Globals.hwMap.get(CRServo.class, name).getController(),
                Globals.hwMap.get(CRServo.class, name).getPortNumber(), direction);
        power = 0;
        lastPower = 0;
    }

    public BluCRServo(String name) {
        this(name, Direction.FORWARD);
    }

    @Override
    public void init() {
        super.setPower(0);
        setPower(0);
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        if(Math.abs(power - lastPower) > 0.03) {
            super.setPower(power);
        }
        lastPower = power;
    }

    @Override
    public void setPower(double power) {
        this.power = Range.clip(power, -1, 1);
    }

    @Override
    public void telemetry() {
        Globals.tele.addLine(name + " power: " + power);
    }
}
