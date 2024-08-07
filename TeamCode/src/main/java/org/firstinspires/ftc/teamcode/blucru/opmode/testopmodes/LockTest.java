package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Lock Test", group = "test")
public class LockTest extends BluLinearOpMode {
    public void initialize() {
        addLocks();
    }

    public void read() {
        if(gamepad1.left_bumper) lock.unlockFrontLockBack();
        else if(gamepad1.right_bumper) lock.unlockAll();
        else lock.lockAll();
    }
}
