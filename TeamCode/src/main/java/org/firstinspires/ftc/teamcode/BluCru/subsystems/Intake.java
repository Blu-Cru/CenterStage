package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem{
    private DcMotorEx intakeMotor;
    private CRServo intakeRoller;
    private IntakeWrist intakeWrist;

    public ElapsedTime intakeTimer;

    public double outtakeRollersPower;
    public double intakeRollersPower;

    public Intake(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(CRServo.class, "intake roller");
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake motor");

        intakeWrist = new IntakeWrist(hardwareMap);

        // set direction
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void init() {
        intakeWrist.init();
        outtakeRollersPower = 0;
        //set all motors to zero power
        intakeRoller.setPower(0);
    }

    public void read() {
        intakeWrist.read();
    }

    public void write() {
        intakeWrist.write();
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
