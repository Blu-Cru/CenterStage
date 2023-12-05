package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BluCru.Constants;

public class Hanger implements Subsystem {
    private DcMotorEx hangerMotor;

    private double PID;
    public double power;
    private int currentPos;
    public int targetPos;

    public Hanger(HardwareMap hardwareMap, Telemetry telemetry) {
        hangerMotor = hardwareMap.get(DcMotorEx.class, "hanger");

        hangerMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void init() {

        targetPos = 0;
        hangerMotor.setPower(0);
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        hangerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {

    }

    public void telemetry(Telemetry telemetry) {

    }

    public void setPower(double power) {
        hangerMotor.setPower(power);
    }

    public int getCurrentPos() {
        return hangerMotor.getCurrentPosition();
    }
}
