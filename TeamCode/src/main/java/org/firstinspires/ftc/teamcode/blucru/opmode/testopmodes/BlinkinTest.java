package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

public class BlinkinTest extends BCLinearOpMode {
    int patternIndex = 0;
    @Override
    public void initialize() {
        addBlinkin();
    }

    @Override
    public void periodic() {
        if (stickyG1.dpad_up) {
            patternIndex++;
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternIndex));
        }

        if (stickyG1.dpad_down) {
            patternIndex--;
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternIndex));
        }
    }

    public void telemetry() {
        telemetry.addData("Pattern Index", patternIndex);
        telemetry.addData("Pattern", RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternIndex));
    }
}
