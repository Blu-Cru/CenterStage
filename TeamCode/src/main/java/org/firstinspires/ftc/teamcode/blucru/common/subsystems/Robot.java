package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Robot {
    private HardwareMap hardwareMap;

    public Outtake outtake;
    public Intake intake;
    public Drivetrain drivetrain;
    public Hanger hanger;

//    public Plane plane;
    public PurplePixelHolder purplePixelHolder;

    private ArrayList<Subsystem> subsystems;

    public Robot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        hanger = new Hanger(hardwareMap);
//        plane = new Plane(hardwareMap);
        purplePixelHolder = new PurplePixelHolder(hardwareMap);

        subsystems = new ArrayList<>();
//        subsystems.add(outtake);
//        subsystems.add(intake);
//        subsystems.add(drivetrain);
//        subsystems.add(hanger);
//        subsystems.add(plane);
//        subsystems.add(purplePixelHolder);
    }

    // initializes subsystems
    public void init() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for(Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void read() {
        // clear bulk cache for bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        for(Subsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for(Subsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public Outtake addOuttake() {
        subsystems.add(outtake);
        return outtake;
    }

    public Intake addIntake() {
        subsystems.add(intake);
        return intake;
    }

    public Drivetrain addDrivetrain() {
        subsystems.add(drivetrain);
        return drivetrain;
    }

    public Hanger addHanger() {
        subsystems.add(hanger);
        return hanger;
    }

//    public Plane addPlane() {
//        subsystems.add(plane);
//        return plane;
//    }

    public PurplePixelHolder addPurplePixelHolder() {
        subsystems.add(purplePixelHolder);
        return purplePixelHolder;
    }

    public void telemetry(Telemetry telemetry) {
        for(Subsystem subsystem : subsystems) {
            subsystem.telemetry(telemetry);
        }
    }
}