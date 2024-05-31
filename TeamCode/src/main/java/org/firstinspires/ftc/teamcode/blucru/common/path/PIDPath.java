package org.firstinspires.ftc.teamcode.blucru.common.path;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;

public class PIDPath implements Path {
    ArrayList<PIDPoint> poseList;
    int index;
    boolean done;

    public PIDPath(ArrayList<PIDPoint> poseList) {
        this.poseList = poseList;
        index = 0;
        done = false;
    }

    @Override
    public void init() {
        index = 0;
    }

    public void follow() {
        Robot.getInstance().drivetrain.pidTo(poseList.get(index).pose);

        if(poseList.get(index).atTarget()) {
            index++;
        }
    }

    public void breakPath() {
        Robot.getInstance().drivetrain.idle();
        done = true;
    }

    public boolean isDone() {
        return index >= poseList.size() || done;
    }
}
