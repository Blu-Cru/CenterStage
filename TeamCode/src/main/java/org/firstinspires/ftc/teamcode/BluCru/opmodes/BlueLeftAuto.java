package org.firstinspires.ftc.teamcode.BluCru.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.states.Side;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.BluCru.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.BluCru.vision.CVMaster;

@Autonomous(name = "Blue Left Auto", group = "BluCru")
public class BlueLeftAuto extends LinearOpMode {
    Robot robot;
    Trajectories trajectories;
    CVMaster cvMaster;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        trajectories = new Trajectories(Alliance.BLUE, Side.CLOSE);
        cvMaster = new CVMaster(hardwareMap, Alliance.BLUE);

        robot.init();
        cvMaster.detectProp();

        waitForStart();
    }
}
