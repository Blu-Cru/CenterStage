package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BluCru.Constants;

@Config
@TeleOp(name = "rollers test", group = "TeleOp")
public class RollersTest extends LinearOpMode {

    String name = "test1";
    String name2 = "test2";
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx test = hardwareMap.get(DcMotorEx.class, name);
        DcMotorEx test2 = hardwareMap.get(DcMotorEx.class, name2);
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        test2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double vert1;
        double vert2;
        waitForStart();
        while(opModeIsActive()) {
            name = Constants.motorTestName;
            test = hardwareMap.get(DcMotorEx.class, name);

            vert1 = -gamepad1.left_stick_y;
            vert2 = -gamepad1.right_stick_y;

            if(Math.abs(vert1) > 0.1) {
                test.setPower(vert1);
            } else {
                test.setPower(0);
            }

            if(Math.abs(vert2) > 0.1) {
                test2.setPower(vert2);
            } else {
                test2.setPower(0);
            }

            telemetry.addData("name", name);
            telemetry.addData("name2", name2);
            telemetry.addData("power 1", test.getPower());
            telemetry.addData("power 2", test2.getPower());
            telemetry.addData("current position 1", test.getCurrentPosition());
            telemetry.addData("current pos 2", test2.getCurrentPosition());
            telemetry.update();
        }
    }
}
