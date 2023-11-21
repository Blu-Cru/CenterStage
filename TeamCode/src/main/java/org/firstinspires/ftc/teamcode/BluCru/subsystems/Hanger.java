package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanger implements Subsystem {
    private DcMotorEx hangerMotor;
    public Hanger(HardwareMap hardwareMap, Telemetry telemetry) {
        hangerMotor = hardwareMap.get(DcMotorEx.class, "hanger");

        hangerMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void init() {
        hangerMotor.setPower(0);
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {

    }
}
