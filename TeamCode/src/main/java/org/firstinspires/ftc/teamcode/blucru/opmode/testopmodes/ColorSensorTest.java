package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

public class ColorSensorTest extends BCLinearOpMode {
    public void periodic() {
    }

    public void initialize() {
        addIntakeColorSensors();
        intakeColorSensors.startReading();
    }

    public void telemetry() {
        intakeColorSensors.telemetry(telemetry);
    }
}
