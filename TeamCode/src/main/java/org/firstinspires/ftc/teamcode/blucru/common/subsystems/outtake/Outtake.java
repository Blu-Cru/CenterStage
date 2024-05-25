package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
            LOW_HEIGHT = 4.3, // inches
            MED_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 2, // inches
            HIGH_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 4,

            MIN_HEIGHT = LOW_HEIGHT - PIXEL_HEIGHT * 1,
            MAX_HEIGHT = LOW_HEIGHT + PIXEL_HEIGHT * 10,

            MAX_TELEOP_TURRET_X = 5.3; // inches

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

    Vector2d targetPos; // (x, z) where x is sideways (turret) and z is up (lift and turret), in inches

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");

        lift = new Lift(hardwareMap);
        lock = new Lock(hardwareMap);
        turret = new Turret(hardwareMap);

        state = State.RETRACT;

        wristRetracted = true;

        targetPos = new Vector2d(0, 0);
    }

    public void init() {
        lift.init();
        lock.init();
        turret.init();
        wrist.setPosition(WRIST_RETRACT);
    }

    public void read() {
        lift.read();
        lock.read();
        turret.read();
    }

    public void write() {
        switch(state) {
            case RETRACT:
                break;
            case OUTTAKE:
                lift.setTargetHeight(targetHeight - turret.getTurretHeightDelta());
                break;
            case MANUAL:
                updateTargetHeightDecel();
                break;
        }
        
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

    public void updateTargetHeightDecel() {
        this.targetHeight = lift.toInches(lift.getTargetPos() + lift.getDecelDelta()) - turret.getTurretHeightDelta();
    }

    public void retractLift() {
        state = State.RETRACT;
        lift.setMotionProfileTargetPos(0);
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
        turret.targetAngle = angleDeg;
    }

    public void setTurretX(double xInches) {
        xInches = Range.clip(xInches, -MAX_TELEOP_TURRET_X, MAX_TELEOP_TURRET_X);
        setTurretAngle(Turret.xToAngle(xInches));
    }

    public void teleOpTurnTurret(double x) {
        double xInches = x * MAX_TELEOP_TURRET_X;
        setTurretX(xInches);
    }

    public double getTurretAngle() {
        return turret.targetAngle;
    }

    public void retractWrist() {
        if(turret.isCentered()) wristRetracted = true;
    }

    public void extendWrist() {
        wristRetracted = false;
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
