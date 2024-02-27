package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "motor test", group = "hardware test")
public class MotorTest extends LinearOpMode {
    public static String name = "hanger";
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx test = hardwareMap.get(DcMotorEx.class, name);
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double vert;
        waitForStart();
        while(opModeIsActive()) {
            test = hardwareMap.get(DcMotorEx.class, name);

            vert = -gamepad1.left_stick_y;

            if(Math.abs(vert) > 0.1) {
                test.setPower(vert);
            } else {
                test.setPower(0);
            }

            telemetry.addData("name", name);
            telemetry.addData("power", test.getPower());
            telemetry.addData("current position", test.getCurrentPosition());
            telemetry.update();
        }
    }
}
