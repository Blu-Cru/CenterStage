package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;

@TeleOp(name = "lift encoder test", group = "test")
public class LiftEncoderTest extends KLinearOpMode {
    @Override
    public void initialize() {
        addOuttake();
        outtake.setManualSlidePower(0);
    }

    @Override
    public void telemetry() {
        outtake.testTelemetry(telemetry);
    }
}
