package org.firstinspires.ftc.teamcode.blucru.common.subsystems.blinkin;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

public class Blinkin implements Subsystem {
    private enum State {
        IDLE,
        INTAKE_FULL,
        INTAKING,
        OUTTAKING,
        ENDGAME
    }
    RevBlinkinLedDriver driver;
    BlinkinPattern currentPattern, lastPattern;
    double intakeFullTime;
    State state;

    public Blinkin(HardwareMap hardwareMap) {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        currentPattern = BlinkinPattern.BREATH_BLUE;
        intakeFullTime = Globals.runtime.seconds();
    }

    public void init() {
        state = State.IDLE;
        driver.setPattern(BlinkinPattern.BREATH_BLUE);
    }

    public void read() {

    }

    public void write() {
        switch(state){
            case IDLE:
                currentPattern = BlinkinPattern.BREATH_BLUE;
                break;
            case INTAKE_FULL:
                doIntakeFull();
                break;
            case INTAKING:
            case OUTTAKING:
                break;
            case ENDGAME:
                doEndgame(Globals.runtime);
                break;
        }

        if(currentPattern != lastPattern) {
            driver.setPattern(currentPattern);
        }
        lastPattern = currentPattern;
    }

    public void startEndgame() {
        state = State.ENDGAME;
    }

    public void idle() {
        state = State.IDLE;
    }

    private void doEndgame(ElapsedTime runtime) {
        if(runtime.seconds() % 1 < 0.2) {
            currentPattern = BlinkinPattern.RED;
        } else {
            currentPattern = BlinkinPattern.BLACK;
        }
    }

    private void doIntakeFull() {
        double time = Globals.runtime.seconds() - intakeFullTime;
        if(time < 1.5) {
            currentPattern = BlinkinPattern.WHITE;
        } else {
            if(time%1.5 < 1) currentPattern = BlinkinPattern.GREEN;
            else currentPattern = BlinkinPattern.RED_ORANGE;
        }
    }

    public void startIntakeFull() {
        state = State.INTAKE_FULL;
        intakeFullTime = Globals.runtime.seconds();
    }

    public void setPattern(BlinkinPattern pattern) {
        driver.setPattern(pattern);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Pattern", currentPattern);
    }
}
