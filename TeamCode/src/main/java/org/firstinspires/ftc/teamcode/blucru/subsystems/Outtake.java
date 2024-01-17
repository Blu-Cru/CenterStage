package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake implements Subsystem{
    public static double WRIST_RETRACT = 0.5;
    // 60 degrees change
    public static double WRIST_OUTTAKE = WRIST_RETRACT + 0.2;

    public static double TURRET_CENTER = 0.5;
    // 90 degrees change
    public static double TURRET_LEFT = TURRET_CENTER - 0.3;
    public static double TURRET_RIGHT = TURRET_CENTER + 0.3;

    public static double BACK_LOCKED = 0.5;
    public static double BACK_UNLOCKED = BACK_LOCKED + 0.1;

    public static double FRONT_LOCKED = 0.5;
    public static double FRONT_UNLOCKED = FRONT_LOCKED - 0.1;

    public final double TURRET_RADIUS = 5.984; // inches

    Servo wrist, turret, backLock, frontLock;
    Lift lift;

    boolean wristRetracted;
    double wristPos;

    public double targetHeight; // inches

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        turret = hardwareMap.get(Servo.class, "turret");
        backLock = hardwareMap.get(Servo.class, "back");
        frontLock = hardwareMap.get(Servo.class, "front");

        wristRetracted = true;
        wristPos = WRIST_RETRACT;
    }

    public void init() {
        lift.init();
        wrist.setPosition(wristPos);
        turret.setPosition(TURRET_CENTER);
        backLock.setPosition(BACK_LOCKED);
        frontLock.setPosition(FRONT_LOCKED);
    }

    public void read() {
        lift.read();
        wristPos = wristRetracted ? WRIST_RETRACT : WRIST_OUTTAKE;
    }

    public void write() {
        lift.write();
        wrist.setPosition(wristPos);
    }

    public void telemetry(Telemetry telemetry) {

    }
}
