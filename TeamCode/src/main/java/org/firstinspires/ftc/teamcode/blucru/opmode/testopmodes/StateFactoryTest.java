package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "state factory test", group = "test")
public class StateFactoryTest extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            telemetry.update();
        }
    }
}
