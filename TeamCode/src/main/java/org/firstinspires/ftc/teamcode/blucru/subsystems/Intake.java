package org.firstinspires.ftc.teamcode.blucru.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake implements Subsystem{
    public static double WRIST_RETRACT_DEG = 80;
    public static double WRIST_INTAKE_DEG = -40;
    public static double WRIST_STACK1_DEG = -30;
    public static double WRIST_STACK3_DEG = -15;
    public static double WRIST_STACK4_DEG = -7;

    public static double POWER = 0.75;

    private DcMotorEx intakeMotor;
    private CRServo intakeRoller;
    private IntakeWrist intakeWrist;

    public ElapsedTime intakeTimer;

    public double intakePower;

    public Intake(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(CRServo.class, "intake roller");
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake motor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        intakeWrist = new IntakeWrist(hardwareMap);
    }

    public void init() {
        intakeWrist.init();
        //set all motors to zero power
        intakeRoller.setPower(0);
    }

    public void read() {
        intakeWrist.read();
    }

    public void write() {
        intakeWrist.write();

        setIntakePower(intakePower);
    }

    public void retractIntakeWrist() {
        setIntakeWristTargetAngle(WRIST_RETRACT_DEG);
    }

    public void downIntakeWrist() {
        setIntakeWristTargetAngle(WRIST_INTAKE_DEG);
    }

    public void setIntakeWristTargetAngle(double targetAngleDeg) {
        intakeWrist.targetAngleDeg = targetAngleDeg;
    }

    public void setIntakePower(double power) {
        intakeRoller.setPower(power * POWER);
        intakeMotor.setPower(power * POWER);
    }

    public double getIntakeRollersPower() {
        return intakeRoller.getPower();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake power", intakePower);
    }
}
