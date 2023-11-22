package org.firstinspires.ftc.teamcode.BluCru.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BluCru.Constants;


@TeleOp(name = "motor test", group = "TeleOp")
public class MotorTest extends LinearOpMode {

    String name = Constants.motorTestName;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx test = hardwareMap.get(DcMotorEx.class, name);
        double vert;
        waitForStart();
        while(opModeIsActive()) {
            name = Constants.motorTestName;
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
