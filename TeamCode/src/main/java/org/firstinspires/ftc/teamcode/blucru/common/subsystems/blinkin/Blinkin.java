package org.firstinspires.ftc.teamcode.blucru.common.subsystems.blinkin;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

public class Blinkin implements Subsystem {
    static final BlinkinPattern IDLE_PATTERN = BlinkinPattern.BREATH_BLUE;
    static final BlinkinPattern ENDGAME_PATTERN = BlinkinPattern.HEARTBEAT_RED;

    private enum State {
        IDLE,
        INTAKE_FULL,
        INTAKING,
        OUTTAKING,
        ENDGAME
    }
    RevBlinkinLedDriver driver;
    State state;

    public Blinkin(HardwareMap hardwareMap) {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void init() {
        state = State.IDLE;
        driver.setPattern(IDLE_PATTERN);
    }

    public void read() {

    }

    public void write() {

    }

    public void setPattern(BlinkinPattern pattern) {
        driver.setPattern(pattern);
    }

    public void telemetry(Telemetry telemetry) {

    }
}
