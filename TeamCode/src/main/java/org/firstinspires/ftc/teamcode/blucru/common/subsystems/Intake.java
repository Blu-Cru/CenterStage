package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake implements Subsystem{
    public static double WRIST_RETRACT_HEIGHT = 4.7; // inches
    public static double WRIST_INTAKE_HEIGHT = -3.1; // inches
    public static double WRIST_STACK1_HEIGHT = -2.3622; // inches

    public static double POWER = 1;

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
        setIntakeWristTargetHeight(WRIST_RETRACT_HEIGHT);
    }

    public void downIntakeWrist() {
        setIntakeWristTargetHeight(WRIST_INTAKE_HEIGHT);
    }

    public void intakeWrist(int stackHeight) {
        stackHeight = Math.max(1, Math.min(4, stackHeight));
        setIntakeWristTargetHeight(WRIST_STACK1_HEIGHT + (stackHeight - 1) * 0.5);
    }

    public void setIntakeWristTargetAngle(double targetAngleDeg) {
        intakeWrist.targetAngleDeg = targetAngleDeg;
    }

    public void setIntakeWristTargetHeight(double targetHeight) {
        intakeWrist.targetAngleDeg = intakeWrist.toDeg(targetHeight);
    }

    public void setIntakePower(double power) {
        intakeRoller.setPower(power * POWER);
        intakeMotor.setPower(power * POWER);
    }

    public double getIntakeRollersPower() {
        return intakeRoller.getPower();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake power", intakeMotor.getPower());
    }
}
