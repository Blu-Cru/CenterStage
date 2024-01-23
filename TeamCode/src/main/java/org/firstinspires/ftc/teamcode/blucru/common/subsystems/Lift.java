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
    public static int YELLOW_POS = 1300;
    public static int RETRACT_POS = 0, LOW_POS = 1200, MED_POS = 1500, HIGH_POS = 1800;
    public static int liftMinPos = 0, liftMaxPos = 1560;
    public static double stallCurrent = 20; // amps
    public static double resetCurrent = 1; // amps
    public final int resetTolerance = 20;
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
    private double ff = kF;

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

        motionProfileTimer = new ElapsedTime();
        motionProfile = new MotionProfile(0, 0, fastVelocity, fastAccel);

        lastTime = System.currentTimeMillis();
    }

    public void read() {
        currentPos = getCurrentPos();
        velocity = Range.clip((currentPos - lastPos) * 1000.0 / dt, -motionProfile.vMax, motionProfile.vMax);

        dt = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();
    }

    public void write() {
        PID = liftPID.calculate(currentPos, targetPos);

        switch(liftState) {
            case AUTO:
                // if lift is down and stalling, reset encoder and set power to 0
                if(currentPos < 2 && getCurrent() > resetCurrent && targetPos == 0) {
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

        lastPos = currentPos;

        setPower(power);
    }

//    public void setMotionProfileTargetHeight(double targetHeight) {
//        setMotionProfileTargetPos((int) (toTicks(targetHeight)));
//    }

//    public void setMotionProfileTargetPos(int targetPos) {
//        setMotionProfile(new MotionProfile(targetPos, currentPos, velocity, fastVelocity, fastAccel));
//    }

//    public void setMotionProfileTargetPos(int targetPos, double vMax, double aMax) {
//        setMotionProfile(new MotionProfile(targetPos, currentPos, velocity, vMax, aMax));
//    }

    public void setMotionProfileConstraints(double maxVelocity, double maxAcceleration) {
        motionProfile.setConstraints(maxVelocity, maxAcceleration);
    }

    public void setMotionProfile(MotionProfile profile) {
        liftState = LiftState.AUTO;
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

    public int toTicks(double inches) {
        return (int) (inches / PULLEY_CIRCUMFERENCE * TICKS_PER_REV);
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