package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

public class LockTest extends BCLinearOpMode {
    public void initialize() {
        addLocks();
    }

    public void read() {
        if(gamepad1.left_bumper) locks.unlockFrontLockBack();
        else if(gamepad1.right_bumper) locks.unlockAll();
        else locks.unlockAll();
    }
}
