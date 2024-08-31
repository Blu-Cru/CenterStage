package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class BluMotor extends DcMotorImpl implements BluHardwareDevice {
    public static double RING_BUFFER_MILLIS = 30;

    String name;
    double power = 0, lastPower = 0;
    double encoderTicks = 0, vel = 0;
    double offsetTicks = 0;
    boolean useEncoder = false;

    public BluMotor(String name, DcMotor motor, Direction direction, boolean useEncoder, ZeroPowerBehavior zeroPowerBehavior) {
        super(motor.getController(), motor.getPortNumber(), direction);
        this.name = name;
        this.useEncoder = useEncoder;
        super.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPower(double power) {
        // clip power to -1 to 1
        this.power = Range.clip(power, -1, 1);
    }

    public void init() {
        super.setPower(0);
        resetEncoder();
        offsetTicks = 0;
    }

    public void read() {
        // only update if encoder is being used
        if(useEncoder) {
            encoderTicks = super.getCurrentPosition();
            // TODO: velocity calculation, use ring buffer
        }
    }

    public void write() {
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.02)
            super.setPower(power);

        lastPower = power;
    }

    public void resetEncoder() {
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setCurrentPosition(double pos) {
        offsetTicks = pos - encoderTicks;
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addData(name + " power", power);
        if(useEncoder) {
            tele.addLine(name + " pos: " + encoderTicks);
            tele.addLine(name + " vel: " + vel);
        }
    }
}
