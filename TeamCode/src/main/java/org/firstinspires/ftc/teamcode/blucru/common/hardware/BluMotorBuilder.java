package org.firstinspires.ftc.teamcode.blucru.common.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class BluMotorBuilder {
    String name;
    boolean reversed, useEncoder;
    double conversionFactor;

    public BluMotorBuilder(String name) {
        this.name = name;
        this.reversed = false;
        this.useEncoder = false;
        this.conversionFactor = 1;
    }

    public BluMotorBuilder reverse() {
        this.reversed = true;
        return this;
    }

    public BluMotorBuilder useEncoder() {
        this.useEncoder = true;
        return this;
    }

    public BluMotorBuilder setConversionFactor(double conversionFactor) {
        this.conversionFactor = conversionFactor;
        return this;
    }

    public BluMotor build() {
        DcMotor motor = Globals.hwMap.get(DcMotor.class, name);
        DcMotorSimple.Direction direction = reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        return new BluMotor(name, motor, direction, useEncoder, conversionFactor);
    }
}
