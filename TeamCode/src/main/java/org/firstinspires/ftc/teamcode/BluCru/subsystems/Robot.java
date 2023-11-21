package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Lift lift;

    private ArrayList<Subsystem> subsystems;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        lift = new Lift(hardwareMap, telemetry);

        subsystems = new ArrayList<>();
        subsystems.add(lift);
    }

    // initializes subsystems
    public void init() {
        for(Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void update() {
        for(Subsystem subsystem : subsystems) {
            subsystem.update();
        }
        telemetry.update();
    }
}
