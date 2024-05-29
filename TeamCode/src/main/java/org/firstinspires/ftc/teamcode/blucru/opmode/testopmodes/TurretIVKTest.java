package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Config
@TeleOp(name = "Turret ivk test", group = "test")
public class TurretIVKTest extends BCLinearOpMode {
    public static double TARGET_Y = 36;
    @Override
    public void initialize() {
        addDrivetrain(true);
        addOuttake();
        addCVMaster();
    }

    @Override
    public void onStart() {
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        if(stickyG1.b) CommandScheduler.getInstance().schedule(new OuttakeExtendCommand(1));
        if(stickyG1.a) CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());

        turret.setGlobalY(TARGET_Y);

        drivetrain.updateAprilTags(cvMaster.tagDetector);
    }
}
