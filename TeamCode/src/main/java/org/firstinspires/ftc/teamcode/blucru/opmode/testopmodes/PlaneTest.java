package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Plane;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PlaneTest extends LinearOpMode {
    Robot robot;
    Plane plane;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();

    }
}
