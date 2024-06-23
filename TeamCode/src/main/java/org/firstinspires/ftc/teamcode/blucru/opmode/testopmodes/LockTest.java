package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Lock Test", group = "test")
public class LockTest extends BCLinearOpMode {
    public void initialize() {
        addLocks();
    }

    public void read() {
        if(gamepad1.left_bumper) lock.unlockFrontLockBack();
        else if(gamepad1.right_bumper) lock.unlockAll();
        else lock.lockAll();
    }
}
