package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;

@Config
public class Outtake implements Subsystem{
    public static double WRIST_RETRACT = 0.61;
    // 60 degrees change
    public static double WRIST_OUTTAKE = WRIST_RETRACT - 0.28;

    public static double BACK_UNLOCKED = 0.7;
    public static double BACK_LOCKED = BACK_UNLOCKED - 0.2;

    public static double FRONT_UNLOCKED = 0.7;
    public static double FRONT_LOCKED = FRONT_UNLOCKED - 0.15;

    public static double PIXEL_HEIGHT = 2.6; // inches
    public static double LOW_HEIGHT = 4.3; // inches
    public static double MED_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 2; // inches
    public static double HIGH_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 4;

    public static double MIN_HEIGHT = LOW_HEIGHT;
    public static double MAX_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 10;

    public static int LIFT_WRIST_CLEAR_POS = 500;
    public static int LIFT_INTAKE_READY_POS = 50;

    Servo wrist, backLock, frontLock;
    public Lift lift;

    public Turret turret;

    public boolean outtaking;

    public boolean wristRetracted;

    public double targetHeight; // inches

    public boolean frontLocked;
    public boolean backLocked;

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
    }

    public void init() {
        lift.init();
        turret.init();
        wrist.setPosition(WRIST_RETRACT);
        backLock.setPosition(BACK_LOCKED);
        frontLock.setPosition(FRONT_LOCKED);
    }

    public void read() {
        if(outtaking) {
            lift.setTargetPos(lift.toTicks(targetHeight - turret.getTurretHeightDelta()));
        }

        lift.read();
        turret.read();
    }

    public void write() {
        lift.write();
        turret.write();

        // write wrist position
        if(wristRetracted && wrist.getPosition() != WRIST_RETRACT) wrist.setPosition(WRIST_RETRACT);
        else if(!wristRetracted && wrist.getPosition() != WRIST_OUTTAKE) wrist.setPosition(WRIST_OUTTAKE);

        // write back lock position
        if(backLocked && backLock.getPosition() != BACK_LOCKED) backLock.setPosition(BACK_LOCKED);
        else if(!backLocked && backLock.getPosition() != BACK_UNLOCKED) backLock.setPosition(BACK_UNLOCKED);

        // write front lock position
        if(frontLocked && frontLock.getPosition() != FRONT_LOCKED) frontLock.setPosition(FRONT_LOCKED);
        else if(!frontLocked && frontLock.getPosition() != FRONT_UNLOCKED) frontLock.setPosition(FRONT_UNLOCKED);
    }

    public void setManualSlidePower(double power) {
        lift.power = power;
        targetHeight = targetHeight + lift.toInches(lift.getDecelDelta());
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = Range.clip(targetHeight, MIN_HEIGHT, MAX_HEIGHT);
        this.lift.setTargetPos((int) lift.toTicks(targetHeight - turret.getTurretHeightDelta()));
    }

    // increase target height by one pixel height
    public void incrementTargetHeight(int pixels) {
        setTargetHeight(targetHeight + pixels * PIXEL_HEIGHT);
    }

    public void updateTargetHeight() {
        this.targetHeight = lift.toInches(lift.getTargetPos()) - turret.getTurretHeightDelta();
    }

    public void retractLift() {
        outtaking = false;
        lift.liftState = LiftState.MoPro;
        lift.setMotionProfileTargetPos(0);
    }

    public boolean liftIntakeReady() {
        return lift.getCurrentPos() < LIFT_INTAKE_READY_POS;
    }

    public void centerTurret() {
        setTurretAngle(270);
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

    public void unlockFrontLockBack() {
        frontLocked = false;
        backLocked = true;
    }

    public Lift getLift() {
        return lift;
    }

    public Turret getTurret() {
        return turret;
    }

    public void telemetry(Telemetry telemetry) {
        lift.telemetry(telemetry);
        turret.telemetry(telemetry);
        telemetry.addData("wrist retracted", wristRetracted);
        telemetry.addData("target height", targetHeight);
    }

    public void testTelemetry(Telemetry telemetry) {
        lift.testTelemetry(telemetry);
        telemetry.addData("wrist pos", wrist.getPosition());
    }
}
