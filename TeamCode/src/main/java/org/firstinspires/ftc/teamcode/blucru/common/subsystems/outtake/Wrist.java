package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Wrist implements Subsystem {
    enum State{
        RETRACT,
        OUTTAKE,
        BACKSTAGE
    }

    public static double WRIST_RETRACT = 0.31,
            WRIST_OUTTAKE = WRIST_RETRACT + 0.44,
            WRIST_BACKSTAGE = WRIST_RETRACT + 0.6;

    Servo wrist;
    State state;

    double position;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        state = State.RETRACT;
    }

    public void init() {
        wrist.setPosition(WRIST_RETRACT);
    }

    public void read() {}

    public void write() {
        switch(state) {
            case RETRACT:
                position = WRIST_RETRACT;
                break;
            case OUTTAKE:
                position = WRIST_OUTTAKE;
                break;
            case BACKSTAGE:
                position = WRIST_BACKSTAGE;
                break;
        }

        if(Math.abs(wrist.getPosition() - position) > 0.01) {
            wrist.setPosition(position);
        }
    }

    public void extend() {
        state = State.OUTTAKE;
    }

    public void retract() {
        state = State.RETRACT;
    }

    public void backstage() {
        state = State.BACKSTAGE;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Wrist state", state);
    }
}
