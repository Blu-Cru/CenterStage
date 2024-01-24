package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.OuttakeState;

@Config
public class Outtake implements Subsystem{
    public static double WRIST_RETRACT = 0.53;
    // 60 degrees change
    public static double WRIST_OUTTAKE = WRIST_RETRACT - 0.26;

    public static double BACK_UNLOCKED = 0.92;
    public static double BACK_LOCKED = BACK_UNLOCKED - 0.28;

    public static double FRONT_UNLOCKED = 0.85;
    public static double FRONT_LOCKED = FRONT_UNLOCKED - 0.28;

    public static double LOW_HEIGHT = 4.0; // inches
    public static double MED_HEIGHT = 8.0; // inches
    public static double HIGH_HEIGHT = 12.0;

    public static int LIFT_WRIST_CLEAR_POS = 300;
    public static int LIFT_INTAKE_READY_POS = 100;

    Servo wrist, backLock, frontLock;
    public Lift lift;

    Turret turret;
    private double lastTurretDelta;

    public boolean outtaking;

    public boolean wristRetracted;
    double wristPos;

    public double targetHeight; // inches
    private double lastTargetHeight;

    public boolean frontLocked;
    public boolean backLocked;
    double backLockPos;
    double frontLockPos;

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        backLock = hardwareMap.get(Servo.class, "back lock");
        frontLock = hardwareMap.get(Servo.class, "front lock");

        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);

        outtaking = false;

        frontLocked = true;
        backLocked = true;

        wristRetracted = true;
        wristPos = WRIST_RETRACT;
    }

    public void init() {
        lift.init();
        turret.init();
        wrist.setPosition(wristPos);
        backLock.setPosition(BACK_LOCKED);
        frontLock.setPosition(FRONT_LOCKED);
    }

    public void read() {
        wristPos = wristRetracted ? WRIST_RETRACT : WRIST_OUTTAKE;
        backLockPos = backLocked ? BACK_LOCKED : BACK_UNLOCKED;
        frontLockPos = frontLocked ? FRONT_LOCKED : FRONT_UNLOCKED;

        if(outtaking) {
            lift.setTargetPos(lift.toTicks(targetHeight - turret.getTurretHeightDelta()));
        }

        lastTargetHeight = targetHeight;
        lastTurretDelta = turret.getTurretHeightDelta();

        lift.read();
        turret.read();
    }

    public void write() {
        lift.write();
        turret.write();
        wrist.setPosition(wristPos);
        backLock.setPosition(backLockPos);
        frontLock.setPosition(frontLockPos);
    }

    public void setManualSlidePower(double power) {
        lift.power = power;
        targetHeight = targetHeight + lift.toInches(lift.getDecelDelta());
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
        this.lift.setTargetPos((int) lift.toTicks(targetHeight - turret.getTurretHeightDelta()));
    }

    public void updateTargetHeight() {
        this.targetHeight = lift.toInches(lift.targetPos) - turret.getTurretHeightDelta();
    }

    public boolean liftIntakeReady() {
        return lift.getCurrentPos() < LIFT_INTAKE_READY_POS;
    }

    public void setTurretAngle(double angleDeg) {
        turret.targetAngle = angleDeg;
    }

    public double getTurretAngle() {
        return turret.targetAngle;
    }

    public void toggleWrist() {
        wristRetracted = !wristRetracted;
    }

    public void retractWrist() {
        wristRetracted = true;
    }

    public void extendWrist() {
        wristRetracted = false;
    }

    public void lock() {
        frontLocked = true;
        backLocked = true;
    }

    public void unlock() {
        frontLocked = false;
        backLocked = false;
    }

    public void lockFront() {
        frontLocked = true;
        backLocked = false;
    }

    public void lockBack() {
        frontLocked = false;
        backLocked = true;
    }

    public void telemetry(Telemetry telemetry) {
        lift.telemetry(telemetry);
        turret.telemetry(telemetry);
        telemetry.addData("wrist retracted", wristRetracted);
        telemetry.addData("wrist pos", wristPos);
        telemetry.addData("target height", targetHeight);
    }
}
