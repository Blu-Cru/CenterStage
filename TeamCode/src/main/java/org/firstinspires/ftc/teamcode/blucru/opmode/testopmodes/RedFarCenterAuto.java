package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name = "red far center auto", group = "test")
public class RedFarCenterAuto extends BCLinearOpMode {
//    ArrayList<TrajectorySequence> trajectoryList
    @Override
    public void initialize() {
        addDrivetrain(false);
        addIntake();
        addOuttake();
        addPurplePixelHolder();
    }
}
