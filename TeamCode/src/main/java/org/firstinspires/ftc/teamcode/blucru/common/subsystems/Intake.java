package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake implements Subsystem{
    public static double WRIST_RETRACT_HEIGHT = 4.7; // inches
    public static double WRIST_INTAKE_HEIGHT = -1.8; // inches
    public static double WRIST_STACK1_HEIGHT = -1.2; // inches
    public static double WRIST_AUTO_READY_HEIGHT = 2.5; // inches

    public static double MAX_POWER = 1;
    double lastPower;

    private DcMotorEx intakeMotor;
    private CRServo intakeRoller;
    private IntakeWrist intakeWrist;

    double intakePower;

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
        lastPower = 0;
    }

    public void read() {
        intakeWrist.read();
    }

    public void write() {
        intakeWrist.write();

        setPower(intakePower);
    }

    public void retractIntakeWrist() {
        setIntakeWristTargetHeight(WRIST_RETRACT_HEIGHT);
    }

    public void downIntakeWrist() {
        setIntakeWristTargetHeight(WRIST_INTAKE_HEIGHT);
    }

    public void dropToStack(int stackHeight) {
        setIntakeWristTargetHeight(getTargetHeight(stackHeight));
    }

    public double getTargetHeight(int stackHeight) {
        if(stackHeight == 0) {
            return Intake.WRIST_INTAKE_HEIGHT;
        } else {
            stackHeight = Math.max(0, Math.min(4, stackHeight));
            return WRIST_STACK1_HEIGHT + (stackHeight - 1) * 0.5;
        }
    }

    public void setIntakeWristTargetAngle(double targetAngleDeg) {
        intakeWrist.targetAngleDeg = targetAngleDeg;
    }

    public void setIntakeWristTargetHeight(double targetHeight) {
        setIntakeWristTargetAngle(intakeWrist.toDeg(targetHeight));
    }

    public void setIntakePower(double power) {
        intakePower = power * MAX_POWER;
    }

    private void setPower(double power) {
        power = Range.clip(power, -1, 1);
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.02) {
            intakeRoller.setPower(power);
            intakeMotor.setPower(power);
            lastPower = power;
        }
    }

    public double getIntakeRollersPower() {
        return intakeRoller.getPower();
    }

    public IntakeWrist getIntakeWrist() {
        return intakeWrist;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake power", intakePower);
    }
}
