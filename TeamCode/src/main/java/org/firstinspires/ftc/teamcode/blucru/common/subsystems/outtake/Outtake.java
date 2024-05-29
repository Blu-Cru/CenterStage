package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Outtake implements Subsystem {
    public static double
            WRIST_RETRACT = 0.68,
            WRIST_OUTTAKE = WRIST_RETRACT - 0.32,

            PIXEL_HEIGHT = 2.6, // inches
            DUNK_HEIGHT = PIXEL_HEIGHT * 0.5,
            LOW_HEIGHT = 3.2, // inches
            MED_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 2, // inches
            HIGH_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 4,

            MIN_HEIGHT = LOW_HEIGHT - PIXEL_HEIGHT * 1,
            MAX_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 10,

            MAX_TURRET_X = 5.3; // inches

    enum State {
        RETRACT,
        OUTTAKE,
        MANUAL
    }

    Servo wrist;
    public Lift lift;
    public Lock lock;
    public Turret turret;

    State state;
    State stateBeforeManual;

    public boolean wristRetracted;

    public double targetHeight; // inches
    double dunkHeight;

    double timeWristExtended;

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");

        lift = new Lift(hardwareMap);
        lock = new Lock(hardwareMap);
        turret = new Turret(hardwareMap);

        state = State.RETRACT;

        wristRetracted = true;
        dunkHeight = 0;
    }

    public void init() {
        lift.init();
        lock.init();
        turret.init();
        wrist.setPosition(WRIST_RETRACT);
    }

    public void read() {
        switch(state) {
            case RETRACT:
                break;
            case OUTTAKE:
                lift.setTargetHeight(targetHeight - turret.getTurretHeightDelta() - dunkHeight);
                if(System.currentTimeMillis() - timeWristExtended < 150 || wristRetracted) centerTurret();
                break;
            case MANUAL:
                updateTargetHeight();
                break;
        }

        lift.read();
        lock.read();
        turret.read();
    }

    public void write() {
        lift.write();
        lock.write();
        turret.write();

        // write wrist position
        if(wristRetracted && wrist.getPosition() != WRIST_RETRACT) wrist.setPosition(WRIST_RETRACT);
        else if(!wristRetracted && wrist.getPosition() != WRIST_OUTTAKE) wrist.setPosition(WRIST_OUTTAKE);
    }

    public void setManualSlidePower(double power) {
        if(state != State.MANUAL) {
            stateBeforeManual = state;
            state = State.MANUAL;
        }
        lift.setManualPower(power);
    }

    public void stopManualSlide() {
        state = stateBeforeManual;
        lift.stopManual();
    }

    public void setTargetHeight(double targetHeight) {
        this.state = State.OUTTAKE;
        this.targetHeight = Range.clip(targetHeight, MIN_HEIGHT, MAX_HEIGHT);
        this.lift.setTargetPos(lift.toTicks(targetHeight - turret.getTurretHeightDelta()));
    }

    public void setTargetPixelHeight(double pixels) {
        setTargetHeight(LOW_HEIGHT + pixels * PIXEL_HEIGHT);
    }

    // increment target height by number of pixels
    public void incrementTargetHeight(double pixels) {
        setTargetHeight(targetHeight + pixels * PIXEL_HEIGHT);
    }

    public void updateTargetHeight() {
        this.targetHeight = lift.toInches(lift.getTargetPos() + lift.getDecelDelta()) - turret.getTurretHeightDelta();
    }

    public void retractLift() {
        state = State.RETRACT;
        lift.setMotionProfileTargetPos(0);
        dunkHeight = 0;
    }

    public boolean liftIntakeReady() {
        return lift.intakeReady();
    }

    public boolean liftWristClear() {
        return lift.wristClear();
    }

    public void centerTurret() {
        setTurretAngle(270);
    }

    public void setTurretAngle(double angleDeg) {
        turret.setAngle(angleDeg);
    }

    public void setTurretGlobalY(double yInches) {
        turret.setGlobalY(yInches);
    }

    public void setTurretX(double xInches) {
        turret.setX(xInches);
    }

    public void teleOpTurnTurret(double x) { // from gamepad input
        double xInches = x * MAX_TURRET_X;
        setTurretX(xInches);
    }

    public void setDunkHeight(double input) {
        dunkHeight = DUNK_HEIGHT * input;
    }

    public double getTurretAngle() {
        return turret.getAngle();
    }

    public void toggleWrist() {
        wristRetracted = !wristRetracted;
    }

    public void retractWrist() {
        wristRetracted = true;
    }

    public void extendWrist() {
        wristRetracted = false;
        timeWristExtended = System.currentTimeMillis();
    }

    public Lift getLift() {
        return lift;
    }

    public Turret getTurret() {
        return turret;
    }

    public void lock() {
        lock.lockAll();
    }

    public void unlock() {
        lock.unlockAll();
    }

    public void unlockFrontLockBack() {
        lock.unlockFrontLockBack();
    }

    public void lockFront() {
        lock.lockFront();
    }

    public void resetLock() {
        lock.reset();
    }

    public void telemetry(Telemetry telemetry) {
        lift.telemetry(telemetry);
        lock.telemetry(telemetry);
        turret.telemetry(telemetry);
        telemetry.addData("outtake state:", state);
        telemetry.addData("wrist retracted", wristRetracted);
        telemetry.addData("target height", targetHeight);
    }

    public void testTelemetry(Telemetry telemetry) {
        lift.testTelemetry(telemetry);
        telemetry.addData("wrist pos", wrist.getPosition());
    }
}
