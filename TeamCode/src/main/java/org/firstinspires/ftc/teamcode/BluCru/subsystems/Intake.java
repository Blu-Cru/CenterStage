package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BluCru.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.states.WristState;

public class Intake implements Subsystem{
    public WristState wristState;
    private DcMotorEx intakeRollers;
    private CRServo outtakeRollers;
    private Servo intakeWrist;
    private Servo outtakeWrist;
    private ServoController outtakeWristController;

    public ElapsedTime intakeTimer;

    public double outtakeRollersPower;
    public double intakeRollersPower;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        wristState = WristState.RETRACT;

        outtakeWrist = hardwareMap.get(Servo.class, "outtake wrist");
        outtakeWristController = outtakeWrist.getController();
        outtakeRollers = hardwareMap.get(CRServo.class, "outtake rollers");
        intakeRollers = hardwareMap.get(DcMotorEx.class, "intake rollers");

        // intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // set direction
        // intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {
        outtakeRollersPower = 0;
        //set all motors to zero power
        intakeRollers.setPower(0);

        //set brake behavior
        intakeRollers.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // reset motor encoders
        intakeRollers.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intakeRollers.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // set servo powers and positions
        outtakeRollers.setPower(0);
        // intakeWristServo.setPosition(Constants.intakeWristRetractPos);
        outtakeWrist.setPosition(Constants.outtakeWristRetractPos);

        intakeTimer = new ElapsedTime();
    }

    public void update() {
        switch(wristState) {
            case RETRACT:
                setOuttakeWristPosition(Constants.outtakeWristRetractPos);
                break;
            case INTAKE:
                setOuttakeWristPosition(Constants.outtakeWristIntakePos);
                break;
            case OUTTAKE:
                setOuttakeWristPosition(Constants.outtakeWristOuttakePos);
                break;
        }

        outtakeRollers.setPower(outtakeRollersPower);
        intakeRollers.setPower(intakeRollersPower);
    }

    public void setOuttakeWristPosition(double position) {
        // turn on wrist servo
        outtakeWristController.pwmEnable();
        outtakeWrist.setPosition(position);
    }

    public void stopOuttakeWrist() {
        // turn off wrist servo
        outtakeWristController.pwmDisable();
    }

    public void setOuttakeRollersPower(double power) {
        outtakeRollers.setPower(power);
    }

    public double getIntakeRollersPower() {
        return intakeRollers.getPower();
    }

    public void toggleWrist() {
        if(wristState == WristState.RETRACT) {
            wristState = WristState.OUTTAKE;
        } else if (wristState == WristState.OUTTAKE){
            wristState = WristState.RETRACT;
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake Rollers Power", intakeRollers.getPower());
        telemetry.addData("Outtake Rollers Power", outtakeRollers.getPower());
        telemetry.addData("Outtake Wrist Position", outtakeWrist.getPosition());
    }
}
