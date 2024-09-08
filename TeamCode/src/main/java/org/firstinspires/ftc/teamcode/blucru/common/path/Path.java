package org.firstinspires.ftc.teamcode.blucru.common.path;

public interface Path {
    // interface for PID path following
    Path start();

    void run();

    void breakPath();

    boolean isDone();
}
