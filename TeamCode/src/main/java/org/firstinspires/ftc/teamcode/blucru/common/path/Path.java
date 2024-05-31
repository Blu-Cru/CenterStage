package org.firstinspires.ftc.teamcode.blucru.common.path;

public interface Path {
    // interface for both PID path following and RoadRunner path following
    void init();

    void follow();

    void breakPath();

    boolean isDone();
}
