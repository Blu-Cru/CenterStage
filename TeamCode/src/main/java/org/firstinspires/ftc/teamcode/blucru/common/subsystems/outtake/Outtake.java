package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Outtake implements Subsystem {
    public static double
            WRIST_RETRACT = 0.68,
            WRIST_OUTTAKE = WRIST_RETRACT - 0.365,

            PIXEL_HEIGHT = 2.6, // inches
            DUNK_HEIGHT = PIXEL_HEIGHT * 0.5,
            LOW_HEIGHT = 3.6, // inches
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

    Wrist wrist;
    public Lift lift;
    public Lock lock;
    public Turret turret;
    BucketLimitSwitch limitSwitch;

    State state;
    State stateBeforeManual;

    boolean turretIsIVK = false;

    public double targetHeight; // inches
    double dunkHeight;

    double turretGlobalY;

    double timeWristExtended;

    public Outtake(HardwareMap hardwareMap) {
        wrist = new Wrist(hardwareMap);
        lift = new Lift(hardwareMap);
        lock = new Lock(hardwareMap);
        turret = new Turret(hardwareMap);
        limitSwitch = new BucketLimitSwitch(hardwareMap);

        state = State.RETRACT;

        dunkHeight = 0;
        turretGlobalY = 0;
    }

    public void init() {
        // TODO: init turret before wrist to prevent getting stuck
        lift.init();
        lock.init();
        turret.init();
        limitSwitch.init();
        wrist.init();
    }

    public void read() {
        switch(state) {
            case RETRACT:
                break;
            case OUTTAKE:
                lift.setTargetHeight(targetHeight - turret.getTurretHeightDelta() - dunkHeight);
                break;
            case MANUAL:
                updateTargetHeight();
                break;
        }

        lift.read();
        lock.read();
        turret.read();
        limitSwitch.read();
    }

    public void write() {
        if(turretIsIVK && turretSafe()) {
            turret.setGlobalY(turretGlobalY);
        }

        lift.write();
        lock.write();
        turret.write();
        limitSwitch.write();
        wrist.write();
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
        turretIsIVK = false;
    }

    public void setTurretGlobalY(double yInches) {
        turretIsIVK = true;
        turretGlobalY = yInches;

        if(turretSafe()) {
            turret.setGlobalY(yInches);
        }
    }

    public void setTurretX(double xInches) {
        turretIsIVK = false;
        if(turretSafe()) turret.setX(xInches);
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

    public boolean turretSafe() {
        return state == State.OUTTAKE && System.currentTimeMillis() - timeWristExtended > 300 && wrist.state != Wrist.State.RETRACT;
    }

    public boolean wristRetracted() {
        return wrist.state == Wrist.State.RETRACT;
    }

    public void wristRetract() {
        wrist.retract();
    }

    public void wristExtend() {
        timeWristExtended = System.currentTimeMillis();
        wrist.extend();
    }

    public void wristBackstage() {
        timeWristExtended = System.currentTimeMillis();
        wrist.backstage();
    }

    public boolean limitSwitchPressed() {
        return limitSwitch.isPressed();
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

    public void unlock(int numReleased) {
        if(numReleased == 1) unlockFrontLockBack();
        else unlock();
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
        telemetry.addData("Outtake state:", state);
        lift.telemetry(telemetry);
        lock.telemetry(telemetry);
        turret.telemetry(telemetry);
        wrist.telemetry(telemetry);
        limitSwitch.telemetry(telemetry);
        telemetry.addData("outtake state:", state);
        telemetry.addData("target height", targetHeight);
        telemetry.addData("dunk height", dunkHeight);
        telemetry.addData("turret is IVK", turretIsIVK);
    }

    public void testTelemetry(Telemetry telemetry) {
        lift.testTelemetry(telemetry);
    }
}
