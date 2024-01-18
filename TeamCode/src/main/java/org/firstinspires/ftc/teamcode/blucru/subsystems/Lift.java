package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;

@Config
public class Lift implements Subsystem{
    public static double liftP = 0.012, liftI = 0, liftD = 0.0005, liftF = 0.05;
    public static int liftRetractPos = 0, liftLowPos = 1200, liftMidPos = 1500, liftHighPos = 1800;
    public static int liftMinPos = -5, liftMaxPos = 2000;
    public static double stallCurrent = 4.0; // amps
    public final int tolerance = 5; // ticks

    public static double TICKS_PER_REV = 384.5;
    public static double PULLEY_CIRCUMFERENCE = 4.40945; // inches

    public static double fastVelocity = 10000.0, fastAccel = 20000.0;
    public static double slowVelocity = 2500.0, slowAccel = 5000.0;

    public LiftState liftState;
    private DcMotorEx liftMotor;
    private DcMotorEx liftMotor2;
    private PIDController liftPID;

    private double PID;
    private double ff = liftF;

    public double power;
    public int targetPos;
    private int currentPos;
    private int lastPos;

    private MotionProfile motionProfile;
    private ElapsedTime motionProfileTimer;

    private double dt;
    private double lastTime;
    private double velocity;

    public Lift(HardwareMap hardwareMap) {
        // declares motors
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        // set direction
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        liftState = LiftState.AUTO;

        targetPos = 0;
    }

    public void init() {
        setTargetPos(0);
        liftPID = new PIDController(liftP, liftI, liftD);

        //set all motors to zero power
        liftMotor.setPower(0);
        liftMotor2.setPower(0);

        //set brake behavior
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reset motor encoders
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfileTimer = new ElapsedTime();
        motionProfile = new MotionProfile(0, 0, fastVelocity, fastAccel);

        lastTime = System.currentTimeMillis();
    }

    public void read() {
        setTargetPos(motionProfile.calculateTargetPosition(motionProfileTimer.seconds()));
        currentPos = getCurrentPos();
        lastPos = currentPos;
        velocity = (currentPos - lastPos) * 1000.0 / dt;

        dt = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();
    }

    public void write() {
        PID = liftPID.calculate(currentPos, targetPos);

        switch(liftState) {
            case AUTO:
                // if lift is down and stalling, reset encoder and set power to 0
                if(currentPos < 5 && getCurrent() > stallCurrent) {
                    power = 0;
                    resetEncoder();
                    setMotionProfile(new MotionProfile(0, 0));
                } else if (getCurrent() > stallCurrent) {
                    setMotionProfile(new MotionProfile(currentPos - inverseP(power), currentPos, 0, slowVelocity, slowAccel));
                } else if (Math.abs(targetPos - currentPos) < tolerance) {
                    power = ff;
                } else {
                    power = PID;
                }
                break;
            case MANUAL:
                // set manual power in opmode
                setMotionProfileTargetPos(currentPos + inverseP(power));
                break;
        }

        setPower(power);
    }

    public void setMotionProfileTargetPos(int targetPos) {
        setMotionProfile(new MotionProfile(targetPos, currentPos, velocity, fastVelocity, fastAccel));
    }

    public void setMotionProfileConstraints(double maxVelocity, double maxAcceleration) {
        motionProfile.setConstraints(maxVelocity, maxAcceleration);
    }

    public void setMotionProfile(MotionProfile profile) {
        liftState = LiftState.AUTO;
        motionProfile = profile;
        motionProfileTimer.reset();
        liftPID.reset();
    }

    public int inverseP(double power) {
        return (int) (power  / liftP);
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setTargetPos(int pos) {
        targetPos = Range.clip(pos, liftMinPos, liftMaxPos);
    }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getTargetPos() {
        return targetPos;
    }

    public int getCurrentPos() {
        return liftMotor.getCurrentPosition();
    }

    public double getCurrent() {
        double current1 = liftMotor.getCurrent(CurrentUnit.AMPS);
        double current2 = liftMotor2.getCurrent(CurrentUnit.AMPS);
        return (current1 + current2) / 2;
    }

    public double toTicks(double inches) {
        return inches / PULLEY_CIRCUMFERENCE * TICKS_PER_REV;
    }

    public double toInches(double ticks) {
        return ticks * PULLEY_CIRCUMFERENCE / TICKS_PER_REV;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("liftState", liftState);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", getCurrentPos());
        telemetry.addData("power", liftMotor.getPower());
        telemetry.addData("velocity", velocity);
        telemetry.addData("current", getCurrent());
        telemetry.addData("motionProfileTimer", motionProfileTimer.seconds());
    }

    public void motionProfileTelemetry(Telemetry telemetry) {
        motionProfile.telemetry(telemetry);
    }
}
