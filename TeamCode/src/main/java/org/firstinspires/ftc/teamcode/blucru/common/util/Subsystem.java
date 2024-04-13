package org.firstinspires.ftc.teamcode.blucru.common.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {

        void init();

        void read();

        void write();

        void telemetry(Telemetry telemetry);
}
