package org.firstinspires.ftc.teamcode.blucru.common.subsystems.blinkin;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeColorSensors;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

import java.util.HashMap;

public class Blinkin implements Subsystem {
    HashMap<IntakeColorSensors.SlotState, BlinkinPattern> pixelPatterns = new HashMap<IntakeColorSensors.SlotState, BlinkinPattern>() {{
        put(IntakeColorSensors.SlotState.WHITE, BlinkinPattern.WHITE);
        put(IntakeColorSensors.SlotState.YELLOW, BlinkinPattern.RED_ORANGE);
        put(IntakeColorSensors.SlotState.PURPLE, BlinkinPattern.VIOLET);
        put(IntakeColorSensors.SlotState.GREEN, BlinkinPattern.GREEN);
    }};

    private enum State {
        IDLE,
        INTAKE_FULL,
        INTAKING,
        OUTTAKING,
        ENDGAME
    }
    RevBlinkinLedDriver driver;
    BlinkinPattern currentPattern, lastPattern;
    BlinkinPattern pixel1 = BlinkinPattern.WHITE;
    BlinkinPattern pixel2 = BlinkinPattern.WHITE;
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
        if(time < 1) {
            currentPattern = BlinkinPattern.WHITE;
        } else if (time < 1.5){
            currentPattern = BlinkinPattern.BLACK;
        } else {
            if(time%1.5 < 1) currentPattern = pixel1;
            else currentPattern = pixel2;
        }
    }

    public void startIntakeFull() {
        state = State.INTAKE_FULL;
        intakeFullTime = Globals.runtime.seconds();
    }

    public void startIntakeFull(IntakeColorSensors sensors) {
        pixel1 = pixelPatterns.get(sensors.frontSlotState);
        pixel2 = pixelPatterns.get(sensors.backSlotState);

        startIntakeFull();
    }

    public void setPattern(BlinkinPattern pattern) {
        driver.setPattern(pattern);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Pattern", currentPattern);
    }
}
