package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Outtake outtake;
    public Intake intake;
    public Drivetrain drivetrain;
    public Hanger hanger;
//    public Plane plane;
    public PurplePixelHolder purplePixelHolder;

    private ArrayList<Subsystem> subsystems;

    public Robot(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        hanger = new Hanger(hardwareMap, telemetry);
//        plane = new Plane(hardwareMap);
        purplePixelHolder = new PurplePixelHolder(hardwareMap);

        subsystems = new ArrayList<>();
        subsystems.add(outtake);
        subsystems.add(intake);
        subsystems.add(drivetrain);
        subsystems.add(hanger);
//        subsystems.add(plane);
        subsystems.add(purplePixelHolder);
    }

    // initializes subsystems
    public void init() {
        for(Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void read() {
        for(Subsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for(Subsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void telemetry(Telemetry telemetry) {
        for(Subsystem subsystem : subsystems) {
            subsystem.telemetry(telemetry);
        }
    }
}
