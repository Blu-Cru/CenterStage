package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake implements Subsystem{
    public static double wristRetract = 0.5;
    // 60 degrees change
    public static double wristOuttake = wristRetract + 0.2;

    public static double turretCenter = 0.5;
    // 90 degrees change
    public static double turretLeft = turretCenter - 0.3;
    public static double turretRight = turretCenter + 0.3;

    public static double backLocked = 0.5;
    public static double backUnlocked = backLocked + 0.1;

    public static double frontLocked = 0.5;
    public static double frontUnlocked = frontLocked - 0.1;

    Servo wrist, turret, backLock, frontLock;
    Lift lift;

    boolean wristRetracted;
    double wristPos;

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        turret = hardwareMap.get(Servo.class, "turret");
        backLock = hardwareMap.get(Servo.class, "back");
        frontLock = hardwareMap.get(Servo.class, "front");

        wristRetracted = true;
        wristPos = wristRetract;
    }

    public void init() {
        lift.init();
        wrist.setPosition(wristPos);
        turret.setPosition(turretCenter);
        backLock.setPosition(backLocked);
        frontLock.setPosition(frontLocked);
    }

    public void read() {
        lift.read();
        wristPos = wristRetracted ? wristRetract : wristOuttake;
    }

    public void write() {
        lift.write();
        wrist.setPosition(wristPos);
    }

    public void telemetry(Telemetry telemetry) {

    }
}
