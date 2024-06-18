package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Blinkin test", group = "test")
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

        if(stickyG1.a) {
            blinkin.startEndgame();
        }

        if(stickyG1.right_stick_button) {
            blinkin.idle();
        }

        if(stickyG1.b) {
            blinkin.startIntakeFull();
        }
    }

    public void telemetry() {
        telemetry.addData("Pattern Index", patternIndex);
        telemetry.addData("Test pattern", RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternIndex));
    }
}
