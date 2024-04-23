package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.hang.Hanger;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeColorSensors;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lock;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.plane.Plane;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.purple.PurplePixelHolder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;

import java.util.ArrayList;

public class Robot {
    private static Robot instance;

    HardwareMap hardwareMap; // reference to hardware

    // all subsystems
    public Outtake outtake;
    public Lift lift;
    public Intake intake;
    public IntakeWrist intakeWrist;
    public Drivetrain drivetrain;
    public Hanger hanger;
    public Plane plane;
    public PurplePixelHolder purplePixelHolder;
    public CVMaster cvMaster;

    public boolean intakingInAuto = false;

    // list of all subsystems
    ArrayList<Subsystem> subsystems;

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot(){
        subsystems = new ArrayList<>();
    }

    public void create(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // initializes subsystems
    public void init() {
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        for(Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void read() {
        // clear bulk cache for bulk reading
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.clearBulkCache();
//        }

        for(Subsystem subsystem : subsystems) {
            subsystem.read();
        }

        if(intakingInAuto && intake.isFull()) {
            drivetrain.breakFollowing();
            intakingInAuto = false;
        }
    }

    public void write() {
        for(Subsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public Outtake addOuttake() {
        outtake = new Outtake(hardwareMap);
        subsystems.add(outtake);
        return outtake;
    }

    public Lift addLift() {
        lift = new Lift(hardwareMap);
        subsystems.add(lift);
        return lift;
    }

    public Intake addIntake() {
        intake = new Intake(hardwareMap);
        subsystems.add(intake);
        return intake;
    }

    public IntakeWrist addIntakeWrist() {
        intakeWrist = new IntakeWrist(hardwareMap);
        subsystems.add(intakeWrist);
        return intakeWrist;
    }

    public IntakeColorSensors addIntakeColorSensors() {
        IntakeColorSensors intakeColorSensors = new IntakeColorSensors(hardwareMap);
        subsystems.add(intakeColorSensors);
        return intakeColorSensors;
    }

    public Drivetrain addDrivetrain(boolean isTeleOp) {
        drivetrain = new Drivetrain(hardwareMap, isTeleOp);
        subsystems.add(drivetrain);
        return drivetrain;
    }

    public Hanger addHanger() {
        hanger = new Hanger(hardwareMap);
        subsystems.add(hanger);
        return hanger;
    }

    public Plane addPlane() {
        plane = new Plane(hardwareMap);
        subsystems.add(plane);
        return plane;
    }

    public PurplePixelHolder addPurplePixelHolder() {
        purplePixelHolder = new PurplePixelHolder(hardwareMap);
        subsystems.add(purplePixelHolder);
        return purplePixelHolder;
    }

    public Lock addLocks() {
        Lock lock = new Lock(hardwareMap);
        subsystems.add(lock);
        return lock;
    }

    public CVMaster addCVMaster(Alliance alliance) {
        cvMaster = new CVMaster(hardwareMap, alliance);
        subsystems.add(cvMaster);
        return cvMaster;
    }

    public void telemetry(Telemetry telemetry) {
        for(Subsystem subsystem : subsystems) {
            subsystem.telemetry(telemetry);
        }

        telemetry.addData("intaking in auto: ", intakingInAuto);
    }

    // call this after every op mode
    public static void kill() {
        instance = null;
    }
}
