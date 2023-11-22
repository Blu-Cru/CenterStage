package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.BluCru.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem{
    private DcMotorEx intakeMotor;
    private CRServo outtakeServo;
    private Servo intakeWristServo;
    private Servo outtakeWristServo;
    private ServoController outtakeWristController;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake wrist");
        outtakeWristController = (ServoControllerEx) outtakeWristServo.getController();

        // set direction
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
        // outtakeServo.setPower(0);
        // intakeWristServo.setPosition(Constants.intakeWristRetractPos);
        outtakeWristServo.setPosition(Constants.outtakeWristIntakePos);
    }

    public void update() {

    }

    public void setOuttakeWristPosition(double position) {
        if(outtakeWristController.getPwmStatus() != ServoControllerEx.PwmStatus.ENABLED) {
            outtakeWristController.pwmEnable();
        }
        outtakeWristServo.setPosition(position);
    }
}
