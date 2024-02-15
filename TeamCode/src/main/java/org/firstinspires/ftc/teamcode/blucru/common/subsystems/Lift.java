package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

@Config
public class Lift implements Subsystem{
    public static double kP = 0.003, kI = 0, kD = 0.0001, kF = 0.04;
    public static int YELLOW_POS = 750, CLEAR_POS = 1100, CYCLE_POS = 1200;
    public static int RETRACT_POS = 0, LOW_POS = 1200, MED_POS = 1500, HIGH_POS = 1800;
    public static int MIN_POS = 0, MAX_POS = 2000;
    public static double stallCurrent = 20; // amps
    public static double resetCurrent = 1; // amps
    public final int tolerance = 5; // ticks

    public static double TICKS_PER_REV = 384.5;
    public static double PULLEY_CIRCUMFERENCE = 4.40945; // inches

    public static double fastVelocity = 8000.0, fastAccel = 12000.0;
    public static double MAX_UP_POWER = 0.85, MAX_DOWN_POWER = -0.8;

    public LiftState liftState;
    private DcMotorEx liftMotor;
    private DcMotorEx liftMotor2;
    private PIDController liftPID;

    private double PID;
    private double ff = kF;

    public double power;
    private int targetPos;
    private int currentPos;

    private MotionProfile motionProfile;
    private ElapsedTime motionProfileTimer;
    ElapsedTime retractTimer;

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
        velocity = 0;
    }

    public void init() {
        setTargetPos(0);
        liftPID = new PIDController(kP, kI, kD);

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

        retractTimer = new ElapsedTime();

        motionProfileTimer = new ElapsedTime();
        motionProfile = new MotionProfile(0, 0, fastVelocity, fastAccel);
    }

    public void read() {
        currentPos = liftMotor.getCurrentPosition();
        velocity = liftMotor.getVelocity();
    }

    public void write() {
        PID = getLiftPID(currentPos, targetPos);

        switch(liftState) {
            case MoPro:
                targetPos = Range.clip(motionProfile.calculateTargetPosition(motionProfileTimer.seconds()), MIN_POS, MAX_POS);
                if(targetPos == 0 && currentPos < 2 && retractTimer.seconds() > 3 && retractTimer.seconds() < 3.5) {
                    power = 0;
                    resetEncoder();
                } else if (Math.abs(targetPos - currentPos) < tolerance) {
                    power = 0;
                } else {
                    PID = getLiftPID(currentPos, targetPos);
                    power = PID;
                }
                break;
            case AUTO:
                // if lift is down and stalling, reset encoder and set power to 0
                if(currentPos < -10 && targetPos == 0) {
                    power = 0;
                    resetEncoder();
//                } else if (getCurrent() > stallCurrent) {
//                    setTargetPos(currentPos - getDecelDelta());
                } else if (Math.abs(targetPos - currentPos) < tolerance) {
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

//    public void setMotionProfileTargetHeight(double targetHeight) {
//        setMotionProfileTargetPos((int) (toTicks(targetHeight)));
//    }

    public void setMotionProfileTargetPos(int targetPos) {
        targetPos = Range.clip(targetPos, MIN_POS, MAX_POS);
        setMotionProfile(new MotionProfile(targetPos, currentPos, velocity, fastVelocity, fastAccel));

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
        if(velocity > 0) {
            return (int) (velocity * velocity / (2 * motionProfile.aMax));
        } else {
            return (int) (velocity * velocity / (-2 * motionProfile.aMax));
        }
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setTargetPos(int pos) {
        liftState = LiftState.AUTO;
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

    public int toTicks(double inches) {
        return (int) (inches / PULLEY_CIRCUMFERENCE * TICKS_PER_REV);
    }

    public double toInches(double ticks) {
        return ticks * PULLEY_CIRCUMFERENCE / TICKS_PER_REV;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("liftState", liftState);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("power", power);
    }

    public void testTelemetry(Telemetry telemetry) {
        telemetry.addData("velocity", velocity);
    }

    public void motionProfileTelemetry(Telemetry telemetry) {
        telemetry.addData("motionProfileTimer", motionProfileTimer.seconds());
        motionProfile.telemetry(telemetry);
    }
}
