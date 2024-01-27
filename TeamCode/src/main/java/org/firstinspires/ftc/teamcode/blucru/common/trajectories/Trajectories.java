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

    ArrayList<TrajectorySequence> trajectoriesFar;
    ArrayList<TrajectorySequence> trajectoriesCenter;
    ArrayList<TrajectorySequence> trajectoriesClose;

    Side side;
    Placements placements;
    Deposits deposits;
    Poses poses;
    Parks parks;

    AutoType autoType;
    ParkType parkType;

    public Trajectories(Alliance alliance, Side side, AutoType autoType, ParkType parkType) {
        trajectoriesFar = new ArrayList<TrajectorySequence>();
        trajectoriesCenter = new ArrayList<TrajectorySequence>();
        trajectoriesClose = new ArrayList<TrajectorySequence>();

        this.side = side;
        this.autoType = autoType;
        this.parkType = parkType;

        if(alliance == Alliance.RED) {
            reflect = 1;
        } else {
            reflect = -1;
        }

        placements = new Placements(reflect);
        deposits = new Deposits(reflect);
        poses = new Poses(reflect);
        parks = new Parks(reflect);
    }

    public ArrayList<TrajectorySequence> buildTrajectoriesFar(Robot robot) {
        trajectoriesFar.clear();

        if(side == Side.CLOSE) {
            trajectoriesFar.add(placements.placementBackdropFar(robot));
            trajectoriesFar.add(deposits.depositFromBackdropFar(robot));

            switch(autoType) {
                case CENTER_CYCLE:
                    trajectoriesFar.add(deposits.cycleCenterFromFar(robot, 3));
                    trajectoriesFar.add(deposits.cycleCenterFromFar(robot, 1));
                    switch (parkType) {
                        case CENTER:
                            trajectoriesFar.add(parks.parkCenterBackdropFar(robot));
                            break;
                        case PERIMETER:
                            trajectoriesFar.add(parks.parkCloseBackdropFar(robot));
                            break;
                        default:
                            break;
                    }
                    break;
                case PERIMETER_CYCLE:
                    trajectoriesFar.add(deposits.cyclePerimeterFromFar(robot, 3));
                    trajectoriesFar.add(deposits.cyclePerimeterFromClose(robot, 1));

                    switch (parkType) {
                        case CENTER:
                            trajectoriesFar.add(parks.parkCenterBackdropClose(robot));
                            break;
                        case PERIMETER:
                            trajectoriesFar.add(parks.parkCloseBackdropClose(robot));
                            break;
                        default:
                            break;
                    }
                    break;
                case PRELOAD:
                    switch (parkType) {
                        case CENTER:
                            trajectoriesFar.add(parks.parkCenterBackdropFar(robot));
                            break;
                        case PERIMETER:
                            trajectoriesFar.add(parks.parkCloseBackdropFar(robot));
                            break;
                        default:
                            break;
                    }
                    break;
            }
        } else {
            switch(autoType) {
                case CENTER_CYCLE:
                    trajectoriesFar.add(placements.placementWingFarForCenter(robot));
                    trajectoriesFar.add(deposits.depositThroughCenterFromWingFar(robot));
                    trajectoriesFar.add(deposits.cycleCenterFromFar(robot, 3));

                    if(parkType == ParkType.CENTER) {
                        trajectoriesFar.add(parks.parkCenterBackdropFar(robot));
                    } else if (parkType == ParkType.PERIMETER) {
                        trajectoriesFar.add(parks.parkCloseBackdropFar(robot));
                    }
                    break;
                case PERIMETER_CYCLE:
                    trajectoriesFar.add(placements.placementWingFarForPerimeter(robot));
                    trajectoriesFar.add(deposits.depositThroughPerimeterFromWingFar(robot));
                    trajectoriesFar.add(deposits.cyclePerimeterFromCenter(robot, 2));

                    switch (parkType) {
                        case CENTER:
                            trajectoriesFar.add(parks.parkCenterBackdropClose(robot));
                            break;
                        case PERIMETER:
                            trajectoriesFar.add(parks.parkCloseBackdropClose(robot));
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    trajectoriesFar.add(placements.placementWingFarForCenter(robot));
                    trajectoriesFar.add(deposits.depositThroughCenterFromWingFar(robot));
                    break;
            }
        }
        return trajectoriesFar;
    }

    public ArrayList<TrajectorySequence>[] build(Robot robot) {
        if(side == Side.CLOSE) {
            trajectoriesFar.add(placements.placementBackdropFar(robot));
            trajectoriesFar.add(deposits.depositFromBackdropFar(robot));

            trajectoriesCenter.add(placements.placementBackdropCenter(robot));
            trajectoriesCenter.add(deposits.depositFromBackdropCenter(robot));

            trajectoriesClose.add(placements.placementBackdropClose(robot));
            trajectoriesClose.add(deposits.depositFromBackdropClose(robot));

            switch(autoType) {
                case CENTER_CYCLE:
                    trajectoriesFar.add(deposits.cycleCenterFromFar(robot, 3));
                    trajectoriesFar.add(deposits.cycleCenterFromFar(robot, 1));

                    trajectoriesCenter.add(deposits.cycleCenterFromCenter(robot, 3));
                    trajectoriesCenter.add(deposits.cycleCenterFromCenter(robot, 1));

                    trajectoriesClose.add(deposits.cycleCenterFromClose(robot, 3));
                    trajectoriesClose.add(deposits.cycleCenterFromFar(robot, 1));
                    break;
                case PERIMETER_CYCLE:
                    trajectoriesFar.add(deposits.cyclePerimeterFromFar(robot, 3));
                    trajectoriesFar.add(deposits.cyclePerimeterFromClose(robot, 1));

                    trajectoriesCenter.add(deposits.cyclePerimeterFromCenter(robot, 3));
                    trajectoriesCenter.add(deposits.cyclePerimeterFromClose(robot, 1));

                    trajectoriesClose.add(deposits.cyclePerimeterFromClose(robot, 3));
                    trajectoriesClose.add(deposits.cyclePerimeterFromClose(robot, 1));
                    break;
                case PRELOAD:
                    break;
            }

            switch(parkType) {
                case NONE:
                    break;
                case CENTER:
//                    trajectoriesFar.add(parks.parkCenterFromFar(robot));
//                    trajectoriesCenter.add(parks.parkCenterFromCenter(robot));
//                    trajectoriesClose.add(parks.parkCenterFromClose(robot));
                    break;
            }
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
