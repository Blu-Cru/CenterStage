package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lock implements Subsystem{
    public static double
            FLAT = 0.7,
            LOCKED_FRONT = FLAT + toTicks(45),
            LOCK_BACK_POS = FLAT - toTicks(45),
            UNLOCKED_BACK = FLAT - toTicks(180);

    enum LockState {
        LOCKED_FRONT,
        LOCKED_BACK,
        UNLOCKED_BACK,
        UNLOCKED_FRONT
    }

    ServoImplEx lock;
    LockState lockState;
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
        switch(lockState) {
            case LOCKED_FRONT:
                lockPos = LOCKED_FRONT;
                break;
            case LOCKED_BACK:
                lockPos = LOCK_BACK_POS;
                break;
            case UNLOCKED_BACK:
                lockPos = UNLOCKED_BACK;
                break;
            case UNLOCKED_FRONT:
                lockPos = FLAT;
                break;
        }
    }

    public void write() {
        if(lock.getPosition() != lockPos) {
            lock.setPosition(lockPos);
        }
    }

    public void lockAll() {
        lockState = LockState.LOCKED_FRONT;
        lockPos = LOCKED_FRONT;
    }

    public void unlockAll() {
        if(lockState == LockState.LOCKED_BACK) {
            lockState = LockState.UNLOCKED_BACK;
            lockPos = UNLOCKED_BACK;
        } else {
            lockState = LockState.UNLOCKED_FRONT;
            lockPos = FLAT;
        }
    }

    public void lockFront() {
        lockState = LockState.LOCKED_FRONT;
        lockPos = LOCKED_FRONT;
    }

    public void unlockFrontLockBack() {
        lockState = LockState.LOCKED_BACK;
        lockPos = LOCK_BACK_POS;
    }

    public static double toTicks(double degrees) {
        return (degrees / 300.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Lock State", lockState);
    }
}
