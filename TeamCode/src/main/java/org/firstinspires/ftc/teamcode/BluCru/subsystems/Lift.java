package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;

public class Lift implements Subsystem{
    private DcMotorEx liftMotor, liftMotor2;
    private PIDController liftPID;

    private double PID;
    private double ff = Constants.sliderF;

    public int targetPos = 0;
    private int currentPos;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        // declares motors
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "lift2");
        // set direction
        liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {
        setTargetPos(0);
        liftPID = new PIDController(Constants.sliderP, Constants.sliderI, Constants.sliderD);

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
    }

    public void update() {
        targetPos = Range.clip(targetPos, Constants.sliderMinPos, Constants.sliderMaxPos);
        currentPos = getCurrentPos();
        PID = liftPID.calculate(currentPos, targetPos);
        setPower(PID + ff);
    }

    public void setPower(double power) {
        power = Range.clip(power, -0.5, 1);
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setTargetPos(int pos) {
        targetPos = pos;
    }

    public int getTargetPos() {
        return targetPos;
    }

    public int getCurrentPos() {
        return liftMotor.getCurrentPosition();
    }
}
