package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCPDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Lift implements Subsystem{
    public static double
            kP = 0.003, kI = 0, kD = 0.0001, kF = 0.04, // PID values

            stallCurrent = 20, resetCurrent = 1, // amps

            TICKS_PER_REV = 384.5, // ticks
            PULLEY_CIRCUMFERENCE = 4.40945, // inches

            fastVelocity = 10000.0, fastAccel = 13000.0, // ticks per second, ticks per second squared
            MAX_UP_POWER = 0.9, MAX_DOWN_POWER = -0.85;

    public static int
            YELLOW_POS = 750, CLEAR_POS = 1100, CYCLE_POS = 1250, // ticks
            MIN_POS = 0, MAX_POS = 2500,
            PID_TOLERANCE = 5, // ticks
            WRIST_CLEAR_POS = 500,
            INTAKE_READY_POS = 50;

    public LiftState liftState;
    DcMotorEx liftMotor;
    DcMotorEx liftMotor2;
    BCPDController liftPID;

    double PID;
    double ff = kF;

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
        liftMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        liftState = LiftState.PID;

        targetPos = 0;
        targetVelocity = 0;
        currentVelocity = 0;
    }

    public void init() {
        setTargetPos(0);
        liftPID = new BCPDController(kP, kD);

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

        switch(liftState) {
            case MoPro:
                targetPos = (int) Range.clip(motionProfile.getInstantTargetPosition(motionProfileTimer.seconds()), MIN_POS, MAX_POS);
                targetVelocity = motionProfile.getInstantTargetVelocity(motionProfileTimer.seconds());
                if(targetPos == 0 && targetVelocity == 0 && currentPos < 2 && retractTimer.seconds() > 3 && retractTimer.seconds() < 3.1) {
                    power = 0;
                    resetEncoder();
                } else if (Math.abs(targetPos - currentPos) < PID_TOLERANCE) {
                    power = 0;
                } else {
                    PID = getLiftPD(currentPos, targetPos, currentVelocity, targetVelocity);
                    power = PID;
                }
                break;
            case PID:
                // if lift is down and stalling, reset encoder and set power to 0
                if(currentPos < -10 && targetPos == 0) {
                    power = 0;
                    resetEncoder();
//                } else if (getCurrent() > stallCurrent) {
//                    setTargetPos(currentPos - getDecelDelta());
                } else if (Math.abs(targetPos - currentPos) < PID_TOLERANCE) {
                    power = 0;
                } else {
                    power = PID;
                }
                break;
            case MANUAL:
                // set manual power in opmode
                setTargetPos(currentPos + getDecelDelta());
                break;
        }

        setPower(power);
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
        liftState = LiftState.MoPro;
        motionProfile = profile;
        motionProfileTimer.reset();
        liftPID.reset();
    }

    public int getDecelDelta() {
        if(currentVelocity > 0) {
            return (int) (currentVelocity * currentVelocity / (2 * motionProfile.aMax));
        } else {
            return (int) (currentVelocity * currentVelocity / (-2 * motionProfile.aMax));
        }
    }

    public boolean intakeReady() {
        return currentPos < INTAKE_READY_POS;
    }

    public boolean wristClear() {
        return currentPos < WRIST_CLEAR_POS;
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        // if power is significantly different from last power, set power
        if(Math.abs(power - lastPower) > 0.02) {
            liftMotor.setPower(power);
            liftMotor2.setPower(power);
            lastPower = power;
        }
    }

    public void setTargetPos(int pos) {
        liftState = LiftState.PID;
        targetPos = Range.clip(pos, MIN_POS, MAX_POS);
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
        telemetry.addData("liftState", liftState);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("power", power);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("velocity", currentVelocity);
    }

    public void motionProfileTelemetry(Telemetry telemetry) {
        telemetry.addData("motionProfileTimer", motionProfileTimer.seconds());
        motionProfile.telemetry(telemetry);
    }
}
