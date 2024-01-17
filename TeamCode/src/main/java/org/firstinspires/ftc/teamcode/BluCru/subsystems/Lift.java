package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;

@Config
public class Lift implements Subsystem{
    public static double liftP = 0.007, liftI = 0, liftD = 0.0001, liftF = 0.08;
    public static int liftRetractPos = 0, liftLowPos = 1200, liftMidPos = 1500, liftHighPos = 1800;
    public static int liftMinPos = 0, liftMaxPos = 2000;
    public static double stallCurrent = 4.0;

    public static double fastVelocity = 10000.0, fastAccel = 20000.0;
    public static double slowVelocity = 5000.0, slowAccel = 10000.0;

    public LiftState liftState;
    private DcMotorEx liftMotor;
//    private DcMotorEx liftMotor2;
    private PIDController liftPID;

    private double PID;
    private double ff = liftF;

    public double power;
    public int targetPos;
    private int currentPos;
    private int lastPos;

    private ElapsedTime liftStallTimer;

    private LiftMotionProfile liftMotionProfile;
    private ElapsedTime liftMotionProfileTimer;

    private double dt;
    private double lastTime;
    private double velocity;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        // declares motors
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift1");
//        liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        // set direction
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
//        liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        liftState = LiftState.RETRACT;

        targetPos = 0;
    }

    public void init() {
        setTargetPos(0);
        liftPID = new PIDController(liftP, liftI, liftD);

        //set all motors to zero power
        liftMotor.setPower(0);
//        liftMotor2.setPower(0);

        //set brake behavior
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reset motor encoders
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftStallTimer = new ElapsedTime();
        liftMotionProfileTimer = new ElapsedTime();
        liftMotionProfile = new LiftMotionProfile(0, 0, fastVelocity, fastAccel);

        lastTime = System.currentTimeMillis();
    }

    public void update() {
        dt = System.currentTimeMillis() - lastTime;
        currentPos = getCurrentPos();
        velocity = (currentPos - lastPos) * 1000.0 / dt;
        lastPos = currentPos;
        lastTime = System.currentTimeMillis();
        PID = liftPID.calculate(currentPos, targetPos);

        switch(liftState) {
            case RETRACT:
                targetPos = Constants.sliderRetractPos;
                if(2.5 > liftStallTimer.seconds() && liftStallTimer.seconds() > 2 && currentPos < 30) {
                    power = 0;
                    resetEncoder();
                }
                if(currentPos < 10) {
                    power = 0;
                } else {
                    power = PID;
                }
                break;
            case MoPro:
                targetPos = liftMotionProfile.calculateTargetPosition(liftMotionProfileTimer.seconds());
                if(targetPos == 0 && currentPos < 0) {
                    power = 0;
                } else if(Math.abs(targetPos - currentPos) < 10) {
                    power = ff;
                } else {
                    power = PID;
                }
                break;
            case AUTO:
                if(targetPos == 0 && currentPos < 3) {
                    power = 0;
                } else if(Math.abs(targetPos - currentPos) < 10) {
                    power = ff;
                } else {
                    power = PID;
                }
                break;
            case MANUAL:
                // set manual power in opmode
//                targetPos = currentPos + inverseP(power);
                break;
        }

//        setPower(power);
    }

    public void setMotionProfileTargetPosition(int targetPos) {
        liftState = LiftState.MoPro;
        liftMotionProfile = new LiftMotionProfile(targetPos, currentPos, velocity, fastVelocity, fastAccel);
        liftMotionProfileTimer.reset();
        liftPID.reset();
    }

    public void setMotionProfileConstraints(double maxVelocity, double maxAcceleration) {
        liftMotionProfile.setConstraints(maxVelocity, maxAcceleration);
    }

    public void setMotionProfile(LiftMotionProfile profile) {
        liftState = LiftState.MoPro;
        liftMotionProfile = profile;
        liftMotionProfileTimer.reset();
        liftPID.reset();
    }

    public void retract() {
        resetStallTimer();
        liftState = LiftState.RETRACT;
    }

    public int inverseP(double power) {
        return (int) (power  / liftP);
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        liftMotor.setPower(power);
//        liftMotor2.setPower(power);
    }

    public void setTargetPos(int pos) {
        targetPos = pos;
    }

    public void resetStallTimer() {
        liftStallTimer.reset();
    }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getTargetPos() {
        return targetPos;
    }

    public int getCurrentPos() {
        return liftMotor.getCurrentPosition();
    }

//    public double getCurrent() {
//        double current1 = liftMotor.getCurrent(CurrentUnit.AMPS);
//        double current2 = liftMotor2.getCurrent(CurrentUnit.AMPS);
//        return (current1 + current2) / 2;
//    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("liftState", liftState);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", getCurrentPos());
        telemetry.addData("power", liftMotor.getPower());
        telemetry.addData("velocity", velocity);
//        telemetry.addData("current", getCurrent());
        telemetry.addData("stallTimer", liftStallTimer.seconds());
        telemetry.addData("motionProfileTimer", liftMotionProfileTimer.seconds());
    }

    public void motionProfileTelemetry(Telemetry telemetry) {
        liftMotionProfile.telemetry(telemetry);
    }
}
