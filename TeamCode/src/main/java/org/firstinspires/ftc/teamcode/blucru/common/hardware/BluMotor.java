package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

// TODO: write builder class
public class BluMotor implements BluHardwareDevice {
    String name;
    double power = 0, lastPower = 0;
    double pos = 0, vel = 0;
    boolean useEncoder = false;

    DcMotorEx motor;

    public void setPower(double power) {
        // clip power to -1 to 1
        this.power = Range.clip(power, -1, 1);
    }

    public void init() {

    }

    public void read() {
        // only update if encoder is being used
        if(useEncoder) {
            pos = motor.getCurrentPosition();
            vel = motor.getVelocity();
        }
    }

    public void write() {
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.02) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public void reset() {
        Log.i("BluMotor", "Resetting motor " + name);

    }

    public void telemetry() {
        // telemetry.addLine(name + " pos: " + pos);
        // telemetry.addLine(name + " vel: " + vel);
    }
}
