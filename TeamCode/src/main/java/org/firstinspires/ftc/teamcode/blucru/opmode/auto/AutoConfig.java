package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.ParkType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;

public class AutoConfig {
    AutoType autoType;
    ParkType parkType;
    Pose2d startPose;

    public AutoConfig(AutoType autoType, ParkType parkType) {
        this.autoType = autoType;
        this.parkType = parkType;
        Poses.setAlliance(Globals.alliance);
    }
}
