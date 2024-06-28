package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Intake implements Subsystem {
    public static double
            MAX_POWER = 1,
            JAMMED_VELOCITY = 300;

    public static int PURPLE_POS = -77;

    enum IntakeState {
        IDLE,
        UNJAMMING,
        RELEASING_PURPLE
    }

    DcMotorEx intakeMotor;
    CRServo intakeRoller;
    public Dropdown dropdown;
    public IntakeColorSensors intakeColorSensors;

    IntakeState intakeState;
    double intakePower;
    double lastPower, powerBeforeUnjam;
    double startUnjamTime, startIntakeTime;
    boolean wasJustFull;
    double targetPos, velocity;
    public double currentPos;

    public Intake(HardwareMap hardwareMap) {
        intakeRoller = hardwareMap.get(CRServo.class, "intake roller");
        intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake motor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        dropdown = new Dropdown(hardwareMap); // instantiate intake wrist
        intakeColorSensors = new IntakeColorSensors(hardwareMap); // instantiate intake color sensors
        intakeState = IntakeState.IDLE;
    }

    public void init() {
        dropdown.init();
        intakeColorSensors.init();
        resetEncoder();
        //set all motors to zero power
        intakeRoller.setPower(0);
        intakePower = 0;
        lastPower = 0;
        currentPos = 0;
        targetPos = 0;
        velocity = 0;
    }

    public void read() {
        dropdown.read();
        intakeColorSensors.read();

        currentPos = intakeMotor.getCurrentPosition();
        velocity = intakeMotor.getVelocity();
    }

    public void write() {
        dropdown.write();
        intakeColorSensors.write();

        switch (intakeState) {
            case IDLE:
                if(intakePower > 0.1 && lastPower < 0.1) {
                    startIntakeTime = System.currentTimeMillis();
                }

                if(velocity < JAMMED_VELOCITY && System.currentTimeMillis() - startIntakeTime > 500 && intakePower > 0.1) {
                    intakeState = IntakeState.UNJAMMING;
                    powerBeforeUnjam = intakePower;
                    startUnjamTime = System.currentTimeMillis();
                }
                break;
            case UNJAMMING:
                if(System.currentTimeMillis() - startUnjamTime > 300) {
                    intakeState = IntakeState.IDLE;
                    intakePower = powerBeforeUnjam;
                    break;
                } else {
                    intakePower = -1;
                }
                break;
            case RELEASING_PURPLE:
                if(currentPos < PURPLE_POS) {
                    intakeState = IntakeState.IDLE;
                    intakePower = 0;
                }
                break;
        }

        if(intakeState == IntakeState.IDLE && intakePower > 0.1 && lastPower < 0.1) {
            startIntakeTime = System.currentTimeMillis();
        }

        setMotorPower(intakePower);
        lastPower = intakePower;
    }

    public boolean isFull() {
        return intakeColorSensors.isFull();
    }

    public boolean hasOne() {
        return intakeColorSensors.hasOne();
    }

    public boolean isEmpty() {
        return intakeColorSensors.empty();
    }

    public void retractIntakeWrist() {
        dropdown.retract();
    }

    public void dropToGround() {
        dropdown.dropToGround();
    }

    public void dropToStack(int stackHeight) {
        dropdown.dropToStack(stackHeight);
    }

    public void setPower(double power) {
        intakePower = power * MAX_POWER;
        intakeState = IntakeState.IDLE;
    }

    public void startReleasePurple() {
        resetEncoder();
        intakePower = Globals.correctPower(-0.8);
        intakeState = IntakeState.RELEASING_PURPLE;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public void intake() {
        setPower(1);
        startReadingColor();
    }

    private void setMotorPower(double power) {
        power = Range.clip(power, -1, 1);
        // only update if power has changed
        if(Math.abs(power - lastPower) > 0.02) {
            intakeRoller.setPower(power);
            intakeMotor.setPower(power);
            lastPower = power;
        }
    }

    public void resetEncoder() {
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public double getIntakeRollersPower() {
        return intakeRoller.getPower();
    }

    public Dropdown getIntakeWrist() {
        return dropdown;
    }

    public void startReadingColor() {
        intakeColorSensors.startReading();
    }

    public void stopReadingColor() {
        intakeColorSensors.stopReading();
        intakeColorSensors.reset();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("intake state", intakeState);
        telemetry.addData("intake power", intakePower);
        telemetry.addData("intake current pos", currentPos);
        telemetry.addData("intake velocity", velocity);
        intakeColorSensors.telemetry(telemetry);
    }
}
