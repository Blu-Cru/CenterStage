package org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Lift implements Subsystem {
    public static double
            kP = 0.0031, kI = 0, kD = 0.0001, kF = 0.03, // PID values
            kFprop = 0.000001, // not used

            stallCurrent = 20, resetCurrent = 1, // amps

            TICKS_PER_REV = 384.5, // ticks
            PULLEY_CIRCUMFERENCE = 4.40945, // inches

            fastVelocity = 14000.0, fastAccel = 13000.0, // ticks per second, ticks per second squared
            MAX_UP_POWER = 0.9, MAX_DOWN_POWER = -1;

    public static int
            MIN_POS = 0, MAX_POS = 2500,
            PID_TOLERANCE = 2, // ticks
            WRIST_CLEAR_POS = 500,
            INTAKE_READY_POS = 50;

    enum State {
        PID, MotionProfile, MANUAL
    }

    State state;
    DcMotorEx liftMotor;
    DcMotorEx liftMotor2;
    PDController liftPID;

    double PID;

    double power;
    double lastPower;
    int targetPos;
    int currentPos;

    MotionProfile motionProfile;
    ElapsedTime motionProfileTimer;
    ElapsedTime retractTimer;

    double targetVelocity;
    double currentVelocity;

    public Lift(HardwareMap hardwareMap) {
        // declares motors
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        // set direction
        liftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        state = State.PID;

        targetPos = 0;
        targetVelocity = 0;
        currentVelocity = 0;
    }

    public void init() {
        setTargetPos(0);
        liftPID = new PDController(kP, kD);

        //set all motors to zero power
        liftMotor.setPower(0);
        liftMotor2.setPower(0);
        lastPower = 0;

        //set brake behavior
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reset motor encoders
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        retractTimer = new ElapsedTime();

        motionProfileTimer = new ElapsedTime();
        motionProfile = new MotionProfile(0, 0, fastVelocity, fastAccel);
    }

    public void read() {
        currentPos = liftMotor.getCurrentPosition();
        currentVelocity = liftMotor.getVelocity();
    }

    public void write() {
        PID = getLiftPID(currentPos, targetPos);

        switch(state) {
            case MotionProfile:
                double motionProfilePower = getMotionProfilePD(currentPos, currentVelocity);
                if(motionProfile.done(motionProfileTimer.seconds()) && currentPos < 0 && retractTimer.seconds() > 3) {
                    power = 0;
                    resetEncoder();
                    state = State.PID;
                    targetPos = 0;
                } else if (getAbsPosError() < PID_TOLERANCE) {
                    power = 0;
                } else {
                    power = motionProfilePower;
                }
                break;
            case PID:
                if (getAbsPosError() < PID_TOLERANCE) {
                    power = 0;
                } else {
                    power = PID;
                }
                break;
            case MANUAL:
                // set manual power elsewhere
                targetPos = Range.clip(currentPos + getDecelDelta(), MIN_POS, MAX_POS);
//                setTargetPos(currentPos + getDecelDelta()); // update target position
                break;
        }

        setPower(power);
    }

    public double getMotionProfilePD(double currentPose, double currentVelocity) {
        targetPos = (int) Range.clip(motionProfile.getInstantTargetPosition(motionProfileTimer.seconds()), MIN_POS, MAX_POS);
        double targetVel = motionProfile.getInstantTargetVelocity(motionProfileTimer.seconds());
        return getLiftPD(currentPose, targetPos, currentVelocity, targetVel);
    }

    public double getLiftPID(double currentPos, double targetPos) {
        return Range.clip(liftPID.calculate(currentPos, targetPos), MAX_DOWN_POWER, MAX_UP_POWER);
    }

    public double getLiftPD(double currentPos, double targetPos, double currentVelocity, double targetVelocity) {
        return Range.clip(liftPID.calculate(currentPos, targetPos, currentVelocity, targetVelocity), MAX_DOWN_POWER, MAX_UP_POWER);
    }

//    public void setMotionProfileTargetHeight(double targetHeight) {
//        setMotionProfileTargetPos((int) (toTicks(targetHeight)));
//    }

    public void setMotionProfileTargetPos(int targetPos) {
        targetPos = Range.clip(targetPos, MIN_POS, MAX_POS);
        setMotionProfile(new MotionProfile(targetPos, currentPos, currentVelocity, fastVelocity, fastAccel));

        if(targetPos == 0) {
            retractTimer.reset();
        }
    }

//    public void setMotionProfileTargetPos(int targetPos, double vMax, double aMax) {
//        setMotionProfile(new MotionProfile(targetPos, currentPos, velocity, vMax, aMax));
//    }

    public void setMotionProfileConstraints(double maxVelocity, double maxAcceleration) {
        motionProfile.setConstraints(maxVelocity, maxAcceleration);
    }

    public void setMotionProfile(MotionProfile profile) {
        state = State.MotionProfile;
        motionProfile = profile;
        motionProfileTimer.reset();
        liftPID.reset();
    }

    public void setManualPower(double power) {
        state = State.MANUAL;
        this.power = power;
    }

    public void stopManual() {
        state = State.PID;
        this.power = 0;
    }

    public int getDecelDelta() {
        return (int) (currentVelocity * currentVelocity / Math.signum(currentVelocity) * 2 * motionProfile.aMax);
    }

    public boolean intakeReady() {
        return currentPos < INTAKE_READY_POS;
    }

    public boolean wristClear() {
        return currentPos < WRIST_CLEAR_POS;
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        // cache motor powers to prevent unnecessary writes
        if(Math.abs(power - lastPower) > 0.02) {
            liftMotor.setPower(power);
            liftMotor2.setPower(power);
            lastPower = power;
        }
    }

    public void setTargetPos(int pos) {
        state = State.PID;
        int newTargetPos = Range.clip(pos, MIN_POS, MAX_POS);
//        if(newTargetPos != targetPos) liftPID.reset();
        targetPos = newTargetPos;
    }

    public void setTargetHeight(double inches) {
        setTargetPos(toTicks(inches));
    }

    public double getFeedForward() {
        return kFprop * getCurrentPos();
    }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updatePID() {
        liftPID.setP(kP);
        liftPID.setI(kI);
        liftPID.setD(kD);
    }

    public int getTargetPos() {
        return targetPos;
    }

    public int getCurrentPos() {
        return currentPos;
    }

    public int getAbsPosError() {
        return Math.abs(targetPos - currentPos);
    }

    public int toTicks(double inches) { // convert inches to motor ticks
        return (int) (inches / PULLEY_CIRCUMFERENCE * TICKS_PER_REV);
    }

    public double toInches(double ticks) { // convert motor ticks to inches
        return ticks * PULLEY_CIRCUMFERENCE / TICKS_PER_REV;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("lift state", state);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("power", power);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("velocity", currentVelocity);
        telemetry.addData("target velocity", motionProfile.getInstantTargetVelocity(motionProfileTimer.seconds()));
    }

    public void motionProfileTelemetry(Telemetry telemetry) {
        telemetry.addData("motionProfileTimer", motionProfileTimer.seconds());
        motionProfile.telemetry(telemetry);
    }
}
