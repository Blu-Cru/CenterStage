package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Subsystem;

@Config
public class Intake implements Subsystem {
    public static double
            MAX_POWER = 1,
            JAM_CURRENT = 10.0; // if current exceeds this, unjam

    enum IntakeState {
        IDLE,
        UNJAMMING
    }

    DcMotorEx intakeMotor;
    CRServo intakeRoller;
    public Dropdown dropdown;
    public IntakeColorSensors intakeColorSensors;

    IntakeState intakeState;
    double intakePower;
    double lastPower;
    double powerBeforeUnjamming;
    double intakeCurrent;
    double startUnjamTime;
    boolean wasJustFull;

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
        //set all motors to zero power
        intakeRoller.setPower(0);
        lastPower = 0;
    }

    public void read() {
        dropdown.read();
        intakeColorSensors.read();

//        if (intakePower > 0.5 || intakeState == IntakeState.UNJAMMING) {
//            intakeCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);
//        }

//        if (intakeCurrent > JAM_CURRENT) {
//            intakeState = IntakeState.UNJAMMING;
//            intakePower = -1;
//            startUnjamTime = System.currentTimeMillis();
//
//            if(intakeState != IntakeState.UNJAMMING) {
//                powerBeforeUnjamming = intakePower;
//            }
//        }
    }

    public void write() {
        dropdown.write();
        intakeColorSensors.write();

//        if(intakeState == IntakeState.UNJAMMING) {
//            if(System.currentTimeMillis() - startUnjamTime > 500) {
//                intakeState = IntakeState.IDLE; // unjamming for 500ms
//                intakePower = powerBeforeUnjamming;
//            } else {
//                setPower(-1);
//            }
//        } else {
            setPower(intakePower);
//        }
    }

    public boolean isFull() {
        return intakeColorSensors.isFull();
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

    public void setIntakePower(double power) {
        intakePower = power * MAX_POWER;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public void intake() {
        setIntakePower(1);
        startReadingColor();
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
        telemetry.addData("intake power", intakePower);
        telemetry.addData("intake current", intakeCurrent);
        intakeColorSensors.telemetry(telemetry);
    }
}
