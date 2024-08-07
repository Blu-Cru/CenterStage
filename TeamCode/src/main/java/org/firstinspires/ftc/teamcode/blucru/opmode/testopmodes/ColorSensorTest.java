package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "color sensor test", group = "test")
public class ColorSensorTest extends BluLinearOpMode {
    public void periodic() {
        if(stickyG1.a) {
            if(intakeColorSensors.isReading()) {
                intakeColorSensors.stopReading();
            } else {
                intakeColorSensors.startReading();
            }
        }
    }

    public void initialize() {
        addIntakeColorSensors();
        intakeColorSensors.startReading();
    }

    public void telemetry() {
        intakeColorSensors.testTelemetry(telemetry);
    }
}
