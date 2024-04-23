package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

public class Lock implements Subsystem {
    public static double
            FLAT = 0.1,
            LOCKED_FRONT = FLAT - toTicks(20),
            LOCK_BACK_POS = FLAT + toTicks(20),
            PUSH_POS = FLAT + toTicks(150);

    public enum LockState {
        LOCKED_FRONT,
        LOCKED_BACK,
        UNLOCKED,
        PUSH
    }

    ServoImplEx lock;
    public LockState lockState;
    double lockPos;

    public Lock(HardwareMap hardwareMap) {
        lock = hardwareMap.get(ServoImplEx.class, "front lock");
        lock.setPwmRange(new PwmControl.PwmRange(500, 2500)); // make the range its max range of 300 degrees

        lockState = LockState.LOCKED_FRONT;
        lockPos = LOCKED_FRONT;
    }

    public void init() {
        lock.setPosition(lockPos);
    }

    public void read() {

    }

    public void write() {
        switch(lockState) {
            case LOCKED_FRONT:
                lockPos = LOCKED_FRONT;
                break;
            case LOCKED_BACK:
                lockPos = LOCK_BACK_POS;
                break;
            case UNLOCKED:
                lockPos = FLAT;
                break;
            case PUSH:
                lockPos = PUSH_POS;
        }

        if(lock.getPosition() != lockPos) {
            lock.setPosition(lockPos);
        }
    }

    public void lockAll() {
        lockState = LockState.LOCKED_FRONT;
    }

    public void unlockAll() {
        if(lockState == LockState.LOCKED_BACK || lockState == LockState.PUSH) {
            lockState = LockState.PUSH;
        } else {
            lockState = LockState.UNLOCKED;
        }
    }

    public void lockFront() {
        lockState = LockState.LOCKED_FRONT;
    }

    public void unlockFrontLockBack() {
        lockState = LockState.LOCKED_BACK;
    }

    public void reset() {
        lockState = LockState.UNLOCKED;
    }

    public static double toTicks(double degrees) {
        return (degrees / 300.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lock State", lockState);
    }
}
