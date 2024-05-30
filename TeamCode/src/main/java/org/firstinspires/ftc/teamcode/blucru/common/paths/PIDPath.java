package org.firstinspires.ftc.teamcode.blucru.common.paths;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PIDPath implements Path {
    PIDPoint[] poseList;
    int index;
    boolean done;

    public PIDPath(PIDPoint[] poseList) {
        this.poseList = poseList;
        index = 0;
        done = false;
    }

    @Override
    public void init() {
        index = 0;
    }

    public void follow() {
        Robot.getInstance().drivetrain.pidTo(poseList[index].pose);

        if(poseList[index].atTarget()) {
            index++;
        }
    }

    public void breakPath() {
        Robot.getInstance().drivetrain.idle();
        done = true;
    }

    public boolean isDone() {
        return index >= poseList.length || done;
    }
}
