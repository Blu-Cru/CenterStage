package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.states.OuttakeState;

@Config
public class Outtake implements Subsystem{
    public static double WRIST_RETRACT = 0.37;
    // 60 degrees change
    public static double WRIST_OUTTAKE = WRIST_RETRACT - 0.26;

    public static double BACK_LOCKED = 0.5;
    public static double BACK_UNLOCKED = BACK_LOCKED + 0.2;

    public static double FRONT_LOCKED = 0.62;
    public static double FRONT_UNLOCKED = FRONT_LOCKED + 0.22;

    public static double LOW_HEIGHT = 12.0; // inches
    public static double MED_HEIGHT = 15.0; // inches
    public static double HIGH_HEIGHT = 18.0;

    public static int LIFT_WRIST_CLEAR_POS = 600;

    Servo wrist, backLock, frontLock;
    public Lift lift;
    Turret turret;

    public OuttakeState outtakeState;

    public boolean wristRetracted;
    double wristPos;

    public double targetHeight; // inches

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        backLock = hardwareMap.get(Servo.class, "back lock");
        frontLock = hardwareMap.get(Servo.class, "front lock");

        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);

        outtakeState = OuttakeState.RETRACT;

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
        targetHeight = targetHeight + lift.toInches(lift.getPosDelta(power));
    }

    public void retractLift() {
        this.lift.setMotionProfileTargetPos(0);
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
        this.lift.setMotionProfileTargetHeight(targetHeight + turret.getTurretHeightDelta());
    }

    public void telemetry(Telemetry telemetry) {
        lift.telemetry(telemetry);
        turret.telemetry(telemetry);
        telemetry.addData("wrist retracted", wristRetracted);
        telemetry.addData("wrist pos", wristPos);
        telemetry.addData("target height", targetHeight);
    }
}
