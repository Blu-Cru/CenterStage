package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name = "color sensor test", group = "test")
public class ColorSensorTest extends BCLinearOpMode {
    public void periodic() {
    }

    public void initialize() {
        addIntakeColorSensors();
        intakeColorSensors.startReading();
    }
}
