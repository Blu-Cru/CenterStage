package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.ParkType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class Trajectories {
    static double reflect = 1;

    ArrayList<TrajectorySequence>[] trajectories = new ArrayList[3];

    ArrayList<TrajectorySequence> trajectories0;
    ArrayList<TrajectorySequence> trajectories1;
    ArrayList<TrajectorySequence> trajectories2;

    Side side;
    Placements placements;
    Deposits deposits;
    Poses poses;
    Parks parks;

    AutoType autoType;
    ParkType parkType;

    public Trajectories(Alliance alliance, Side side, AutoType autoType, ParkType parkType) {
        trajectories0 = new ArrayList<TrajectorySequence>();
        trajectories1 = new ArrayList<TrajectorySequence>();
        trajectories2 = new ArrayList<TrajectorySequence>();

        this.side = side;
        this.autoType = autoType;
        this.parkType = parkType;

        if(alliance == Alliance.RED) {
            reflect = 1;
        } else {
            reflect = -1;
        }
    }

    public ArrayList<TrajectorySequence>[] build(Robot robot) {
        placements = new Placements(reflect);
        deposits = new Deposits(reflect);
        poses = new Poses(reflect);
        parks = new Parks(reflect);

        if(side == Side.CLOSE) {

        } else {

        }

        return trajectories;
    }


    public Pose2d getStartPose() {
        switch (side) {
            case CLOSE:
                return Poses.BACKDROP_STARTING_POSE;
            case FAR:
                return Poses.WING_STARTING_POSE;
        }
        return null;
    }
}
