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

    public static double BACK_LOCKED = 0.5;
    public static double BACK_UNLOCKED = BACK_LOCKED + 0.2;

    public static double FRONT_LOCKED = 0.5;
    public static double FRONT_UNLOCKED = FRONT_LOCKED - 0.2;

    public static double LOW_HEIGHT = 12.0; // inches
    public static double MED_HEIGHT = 15.0; // inches
    public static double HIGH_HEIGHT = 18.0;

    Servo wrist, backLock, frontLock;
    public Lift lift;
    Turret turret;

    boolean wristRetracted;
    double wristPos;

    public double targetHeight; // inches

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        backLock = hardwareMap.get(Servo.class, "back");
        frontLock = hardwareMap.get(Servo.class, "front");

        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);

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
        lift.read();
        turret.read();
        wristPos = wristRetracted ? WRIST_RETRACT : WRIST_OUTTAKE;
    }

    public void write() {
        lift.write();
        turret.write();
        wrist.setPosition(wristPos);
    }

    public void setManualSlidePower(double power) {
        lift.power = power;
        targetHeight = targetHeight + lift.toInches(lift.inverseP(power));
    }

    public void telemetry(Telemetry telemetry) {
        lift.telemetry(telemetry);
        turret.telemetry(telemetry);
        telemetry.addData("wrist retracted", wristRetracted);
        telemetry.addData("wrist pos", wristPos);
        telemetry.addData("target height", targetHeight);
    }
}
