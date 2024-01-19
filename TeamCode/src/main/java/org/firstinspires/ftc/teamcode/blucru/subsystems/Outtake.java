package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.states.OuttakeState;

@Config
public class Outtake implements Subsystem{
    public static double WRIST_RETRACT = 0.53;
    // 60 degrees change
    public static double WRIST_OUTTAKE = WRIST_RETRACT - 0.26;

    public static double BACK_UNLOCKED = 0.92;
    public static double BACK_LOCKED = BACK_UNLOCKED - 0.28;

    public static double FRONT_UNLOCKED = 0.85;
    public static double FRONT_LOCKED = FRONT_UNLOCKED - 0.28;

    public static double LOW_HEIGHT = 12.0; // inches
    public static double MED_HEIGHT = 15.0; // inches
    public static double HIGH_HEIGHT = 18.0;

    public static int LIFT_WRIST_CLEAR_POS = 600;
    public static int LIFT_INTAKE_READY_POS = 100;

    Servo wrist, backLock, frontLock;
    public Lift lift;

    Turret turret;
    private double lastTurretDelta;

    public OuttakeState outtakeState;

    public boolean wristRetracted;
    double wristPos;

    public double targetHeight; // inches
    private double lastTargetHeight;

    public boolean locked;
    double backLockPos;
    double frontLockPos;

    public Outtake(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        backLock = hardwareMap.get(Servo.class, "back lock");
        frontLock = hardwareMap.get(Servo.class, "front lock");

        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);

        outtakeState = OuttakeState.RETRACT;

        locked = true;

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

        if(outtakeState == OuttakeState.OUTTAKE) {
            if(targetHeight != lastTargetHeight || turret.getTurretHeightDelta() != lastTurretDelta) {
                setTargetHeight(targetHeight);
            }
        }

        wristPos = wristRetracted ? WRIST_RETRACT : WRIST_OUTTAKE;
        backLockPos = locked ? BACK_LOCKED : BACK_UNLOCKED;
        frontLockPos = locked ? FRONT_LOCKED : FRONT_UNLOCKED;

        lastTargetHeight = targetHeight;
        lastTurretDelta = turret.getTurretHeightDelta();
    }

    public void write() {
        lift.write();
        turret.write();
        wrist.setPosition(wristPos);
        backLock.setPosition(backLockPos);
        frontLock.setPosition(frontLockPos);
    }

    public void setManualSlidePower(double power) {
        lift.power = power;
        targetHeight = targetHeight + lift.toInches(lift.getPosDelta(power));
    }

    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
        this.lift.setMotionProfileTargetHeight(targetHeight + turret.getTurretHeightDelta());
    }

    public void updateTargetHeight() {
        this.targetHeight = lift.toInches(lift.targetPos) + turret.getTurretHeightDelta();
    }

    public boolean liftIntakeReady() {
        return lift.getCurrentPos() < LIFT_INTAKE_READY_POS;
    }

    public void setTurretAngle(double angleDeg) {
        turret.targetAngle = angleDeg;
    }

    public double getTurretAngle() {
        return turret.targetAngle;
    }

    public void toggleWrist() {
        wristRetracted = !wristRetracted;
    }

    public void lock() {
        locked = true;
    }

    public void unlock() {
        locked = false;
    }

    public void telemetry(Telemetry telemetry) {
        lift.telemetry(telemetry);
        turret.telemetry(telemetry);
        telemetry.addData("wrist retracted", wristRetracted);
        telemetry.addData("wrist pos", wristPos);
        telemetry.addData("target height", targetHeight);
    }
}
