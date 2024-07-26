package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.AutoReleasePurpleIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Purple Release Test", group = "test")
public class PurpleReleaseTest extends BluLinearOpMode {
    public static double power = 0.8;
    public static int position = 50;

    @Override
    public void initialize() {
        addIntake();
    }

    public void onStart() {
        intake.dropToStack(4);
    }

    boolean releasing = false;

    @Override
    public void periodic() {
        if(stickyG1.a) new AutoReleasePurpleIntakeCommand(0).schedule();

        if(releasing && intake.currentPos < -position) {
            intake.setPower(0);
            releasing = false;
        }

        if(stickyG1.b) {
            intake.resetEncoder();
            intake.setPower(-power);
            releasing = true;
        }
    }
}
