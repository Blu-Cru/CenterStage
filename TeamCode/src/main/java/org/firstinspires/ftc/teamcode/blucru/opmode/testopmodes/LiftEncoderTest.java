package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "lift encoder test", group = "test")
public class LiftEncoderTest extends BCLinearOpMode {
    @Override
    public void initialize() {
        addOuttake();
    }

    @Override
    public void periodic() {
        outtake.lift.liftState = LiftState.MANUAL;
    }

    @Override
    public void telemetry() {
        outtake.testTelemetry(telemetry);
    }
}
