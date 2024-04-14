package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface BCSubsystem extends Subsystem {

        void init();

        void read();

        void write();

        void telemetry(Telemetry telemetry);
}
