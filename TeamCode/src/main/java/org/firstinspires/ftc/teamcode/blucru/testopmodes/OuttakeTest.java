package org.firstinspires.ftc.teamcode.blucru.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.subsystems.Outtake;

public class OuttakeTest extends LinearOpMode {
    Outtake outtake;
    public void runOpMode() throws InterruptedException {
        outtake = new Outtake(hardwareMap);

        outtake.init();

        waitForStart();
    }
}
