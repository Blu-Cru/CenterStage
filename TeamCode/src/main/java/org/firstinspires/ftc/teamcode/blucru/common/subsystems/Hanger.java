package org.firstinspires.ftc.teamcode.blucru.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Hanger implements Subsystem {
    public static double p = 0.005, i = 0, d = 0;

    private DcMotorEx hangerMotor;

    private double PID;
    public double power;
    private int currentPos;
    public int targetPos;

    public Hanger(HardwareMap hardwareMap) {
        hangerMotor = hardwareMap.get(DcMotorEx.class, "hanger");

        hangerMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {

        targetPos = 0;
        hangerMotor.setPower(0);
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        hangerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void read() {
        currentPos = hangerMotor.getCurrentPosition();
    }

    public void write() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("hanger pos", hangerMotor.getCurrentPosition());
    }

    public void setPower(double power) {
        hangerMotor.setPower(power);
    }

    public int getCurrentPos() {
        return hangerMotor.getCurrentPosition();
    }
}
