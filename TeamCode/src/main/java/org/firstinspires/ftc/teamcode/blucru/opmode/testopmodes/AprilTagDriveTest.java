package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

public class AprilTagDriveTest extends BCLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain(true);
        addCVMaster();
        enableFTCDashboard();
    }

    @Override
    public void periodic() {

    }
}
