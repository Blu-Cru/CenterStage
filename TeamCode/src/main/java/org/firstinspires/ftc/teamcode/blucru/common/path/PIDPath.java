package org.firstinspires.ftc.teamcode.blucru.common.path;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public void start() {
        done = false;
        poseList.get(0).start();
        index = 0;
    }

    public void run() {
        Robot.getInstance().drivetrain.pidTo(poseList.get(index).pose);

        if(poseList.get(index).atTarget() && !done) {
            if(index + 1 == poseList.size()) {
                done = true;
            } else {
                index++;
                poseList.get(index).start();
            }
        }
    }

    public void breakPath() {
        Robot.getInstance().drivetrain.idle();
        done = true;
    }

    public boolean failed() {
        return poseList.get(index).failed();
    }

    public boolean isDone() {
        return index >= poseList.size() || done;
    }

    public void telemetry(Telemetry tele) {
        tele.addData("Path index", index);
    }
}
