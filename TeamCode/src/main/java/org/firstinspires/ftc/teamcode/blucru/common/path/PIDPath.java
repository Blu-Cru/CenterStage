package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.ArrayList;
import java.util.HashMap;

public class PIDPath implements Path {
    ArrayList<PathSegment> segmentList;
    HashMap<Integer, ArrayList<Command>> commands;
    int index;
    boolean pathDone;

    public PIDPath(ArrayList<PathSegment> segmentList, HashMap<Integer, ArrayList<Command>> commands) {
        this.segmentList = segmentList;
        this.commands = commands;
        index = 0;
        pathDone = false;
    }

    @Override
    public void start() {
        pathDone = false;
        segmentList.get(0).start();
        index = 0;
    }

    public void run() {
        Robot.getInstance().drivetrain.pidTo(segmentList.get(index).getPose());

        if(segmentList.get(index).isDone() && !pathDone) {
            if(index + 1 == segmentList.size()) {
                pathDone = true;
            } else {
                index++;

                // run the commands associated with the next point
                try {
                    for(Command c : commands.get(index)) {
                        c.schedule();
                    }
                } catch (NullPointerException ignored) {}

                segmentList.get(index).start();
            }
        }
    }

    public void breakPath() {
        Robot.getInstance().drivetrain.idle();
        pathDone = true;
    }

    public boolean failed() {
        return segmentList.get(index).failed();
    }

    public boolean isDone() {
        return index >= segmentList.size() || pathDone;
    }

    public void telemetry(Telemetry tele) {
        tele.addData("Path index", index);
    }
}
