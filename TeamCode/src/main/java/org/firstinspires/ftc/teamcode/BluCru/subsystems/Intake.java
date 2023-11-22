package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.BluCru.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem{
    private DcMotorEx intakeRollers;
    private CRServo outtakeRollers;
    private Servo intakeWrist;
    private Servo outtakeWrist;
    private ServoController outtakeWristController;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        outtakeWrist = hardwareMap.get(Servo.class, "outtake wrist");
        outtakeWristController = outtakeWrist.getController();
        outtakeRollers = hardwareMap.get(CRServo.class, "outtake");

        // intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // set direction
        // intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {
        /*//set all motors to zero power
        intakeMotor.setPower(0);

        //set brake behavior
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // reset motor encoders
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);*/

        // set servo powers and positions
        outtakeRollers.setPower(0);
        // intakeWristServo.setPosition(Constants.intakeWristRetractPos);
        outtakeWrist.setPosition(Constants.outtakeWristRetractPos);
    }

    public void update() {
        // intakeMotor.setPower(intakePower);
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
}
