package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.states.LiftState;

public class Lift implements Subsystem{
    public static double liftP = 0.007, liftI = 0, liftD = 0.0001, liftF = 0.08;

    public LiftState liftState;
    private DcMotorEx liftMotor, liftMotor2;
    private PIDController liftPID;

    private double PID;
    private double ff = liftF;

    public double power;
    public int targetPos;
    private int currentPos;

    private ElapsedTime liftStallTimer;

    private LiftMotionProfile liftMotionProfile;
    private ElapsedTime liftMotionProfileTimer;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        // declares motors
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        // set direction
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        liftState = LiftState.RETRACT;

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

        liftStallTimer = new ElapsedTime();
        liftMotionProfileTimer = new ElapsedTime();

        liftMotionProfile = new LiftMotionProfile(0, 0);
    }

    public void update() {
        targetPos = liftMotionProfile.calculateTargetPosition(liftMotionProfileTimer.seconds());
        currentPos = getCurrentPos();
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
            case AUTO:
                if(targetPos == 0 && currentPos < 10) {
                    power = 0;
                } else if(Math.abs(targetPos - currentPos) < 10) {
                    power = ff;
                } else {
                    power = PID;
                }
                break;
            case MANUAL:
                // manual power is set in TeleOpStateMachine
                targetPos = currentPos + inverseP(power);
                break;
        }

        setPower(power);
    }

    public void setMotionProfileTargetPosition(int targetPos) {
        liftMotionProfile = new LiftMotionProfile(targetPos, currentPos);
        liftMotionProfileTimer.reset();
        liftPID.reset();
    }

    public void setMotionProfileConstraints(double maxVelocity, double maxAcceleration) {
        liftMotionProfile.setConstraints(maxVelocity, maxAcceleration);
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
        liftMotor2.setPower(power);
    }

    public void setTargetPos(int pos) {
        targetPos = pos;
    }

    public void resetStallTimer() {
        liftStallTimer.reset();
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

    public double getCurrent(DcMotorEx motor) {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("liftState", liftState);
        telemetry.addData("targetPos", targetPos);
        telemetry.addData("power", liftMotor.getPower());
        telemetry.addData("currentPos", getCurrentPos());
        telemetry.addData("stallTimer", liftStallTimer.seconds());
        telemetry.addData("motionProfileTimer", liftMotionProfileTimer.seconds());
    }

    public void motionProfileTelemetry(Telemetry telemetry) {
        liftMotionProfile.telemetry(telemetry);
    }
}
