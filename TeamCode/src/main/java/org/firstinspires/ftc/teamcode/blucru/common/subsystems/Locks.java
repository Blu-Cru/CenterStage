package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Locks implements Subsystem{
    public static double BACK_UNLOCKED = 0.7;
    public static double BACK_LOCKED = BACK_UNLOCKED - 0.2;

    public static double FRONT_UNLOCKED = 0.7;
    public static double FRONT_LOCKED = FRONT_UNLOCKED - 0.15;

    public boolean frontLocked;
    public boolean backLocked;

    Servo backLock;
    Servo frontLock;

    public Locks(HardwareMap hardwareMap) {
        backLock = hardwareMap.get(Servo.class, "back lock");
        frontLock = hardwareMap.get(Servo.class, "front lock");

        frontLocked = true;
        backLocked = true;
    }

    public void init() {
        backLock.setPosition(BACK_LOCKED);
        frontLock.setPosition(FRONT_LOCKED);
    }

    public void read() {

    }

    public void write() {
        // write back lock position
        if(backLocked && backLock.getPosition() != BACK_LOCKED) backLock.setPosition(BACK_LOCKED);
        else if(!backLocked && backLock.getPosition() != BACK_UNLOCKED) backLock.setPosition(BACK_UNLOCKED);

        // write front lock position
        if(frontLocked && frontLock.getPosition() != FRONT_LOCKED) frontLock.setPosition(FRONT_LOCKED);
        else if(!frontLocked && frontLock.getPosition() != FRONT_UNLOCKED) frontLock.setPosition(FRONT_UNLOCKED);
    }

    public void lockAll() {
        frontLocked = true;
        backLocked = true;
    }

    public void unlockAll() {
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

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Front Locked:", frontLocked);
        telemetry.addData("Back Locked:", backLocked);
    }
}
