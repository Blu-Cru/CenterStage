package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.vision.CVMaster;

import java.util.ArrayList;

public class Robot {
    private static Robot instance;

    HardwareMap hardwareMap; // reference to hardware

    // all subsystems
    public Outtake outtake;
    public Intake intake;
    public IntakeWrist intakeWrist;
    public Drivetrain drivetrain;
    public Hanger hanger;
    public Plane plane;
    public PurplePixelHolder purplePixelHolder;
    public CVMaster cvMaster;

    public boolean intakingInAuto = false;

    // list of all subsystems
    ArrayList<BCSubsystem> BCSubsystems;

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot(){
        BCSubsystems = new ArrayList<>();
    }

    public void create(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // initializes subsystems
    public void init() {
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        for(BCSubsystem BCSubsystem : BCSubsystems) {
            BCSubsystem.init();
        }
    }

    public void read() {
        // clear bulk cache for bulk reading
//        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
//            module.clearBulkCache();
//        }

        for(BCSubsystem BCSubsystem : BCSubsystems) {
            BCSubsystem.read();
        }

        if(intakingInAuto && intake.isFull()) {
            drivetrain.breakFollowing();
            intakingInAuto = false;
        }
    }

    public void write() {
        for(BCSubsystem BCSubsystem : BCSubsystems) {
            BCSubsystem.write();
        }
    }

    public Outtake addOuttake() {
        outtake = new Outtake(hardwareMap);
        BCSubsystems.add(outtake);
        return outtake;
    }

    public Intake addIntake() {
        intake = new Intake(hardwareMap);
        BCSubsystems.add(intake);
        return intake;
    }

    public IntakeWrist addIntakeWrist() {
        intakeWrist = new IntakeWrist(hardwareMap);
        BCSubsystems.add(intakeWrist);
        return intakeWrist;
    }

    public IntakeColorSensors addIntakeColorSensors() {
        IntakeColorSensors intakeColorSensors = new IntakeColorSensors(hardwareMap);
        BCSubsystems.add(intakeColorSensors);
        return intakeColorSensors;
    }

    public Drivetrain addDrivetrain(boolean isTeleOp) {
        drivetrain = new Drivetrain(hardwareMap, isTeleOp);
        BCSubsystems.add(drivetrain);
        return drivetrain;
    }

    public Hanger addHanger() {
        hanger = new Hanger(hardwareMap);
        BCSubsystems.add(hanger);
        return hanger;
    }

    public Plane addPlane() {
        plane = new Plane(hardwareMap);
        BCSubsystems.add(plane);
        return plane;
    }

    public PurplePixelHolder addPurplePixelHolder() {
        purplePixelHolder = new PurplePixelHolder(hardwareMap);
        BCSubsystems.add(purplePixelHolder);
        return purplePixelHolder;
    }

    public Lock addLocks() {
        Lock lock = new Lock(hardwareMap);
        BCSubsystems.add(lock);
        return lock;
    }

    public CVMaster addCVMaster(Alliance alliance) {
        cvMaster = new CVMaster(hardwareMap, alliance);
        BCSubsystems.add(cvMaster);
        return cvMaster;
    }

    public void telemetry(Telemetry telemetry) {
        for(BCSubsystem BCSubsystem : BCSubsystems) {
            BCSubsystem.telemetry(telemetry);
        }

        telemetry.addData("intaking in auto: ", intakingInAuto);
    }

    // call this after every op mode
    public static void kill() {
        instance = null;
    }
}
