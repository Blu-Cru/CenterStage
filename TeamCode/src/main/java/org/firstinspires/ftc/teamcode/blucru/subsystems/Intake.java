package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem{
    private DcMotorEx intakeMotor;
    private CRServo intakeRoller;
    private Servo intakeWrist;

    public ElapsedTime intakeTimer;

    public double outtakeRollersPower;
    public double intakeRollersPower;

    public Intake(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(CRServo.class, "intake rollers");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // set direction
        // intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {
        outtakeRollersPower = 0;
        //set all motors to zero power
        intakeRoller.setPower(0);

        intakeTimer = new ElapsedTime();
    }

    public void read() {
        // read encoder values
        // currentPos = intakeMotor.getCurrentPosition();
    }

    public void write() {
        if(intakeTimer.seconds() > 0.25) {
            intakeRoller.setPower(intakeRollersPower);
        } else {
            intakeRoller.setPower(0);
        }
    }

    public void setOuttakeWristPosition(double position) {
        // turn on wrist servo
    }

    public double getIntakeRollersPower() {
        return intakeRoller.getPower();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake Roller Power", intakeRoller.getPower());
    }
}
