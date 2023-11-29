package org.firstinspires.ftc.teamcode.BluCru.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.states.StartSide;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.BluCru.trajectories.Trajectories;

@Autonomous(name = "Red Right Auto", group = "BluCru")
public class RedRightAuto extends LinearOpMode {
    Robot robot;
    Trajectories trajectories;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(telemetry, hardwareMap);
        trajectories = new Trajectories(Alliance.RED, StartSide.CLOSE, robot);

        robot.init();

        waitForStart();
    }
}
