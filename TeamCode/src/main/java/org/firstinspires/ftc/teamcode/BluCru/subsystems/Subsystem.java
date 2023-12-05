package org.firstinspires.ftc.teamcode.BluCru.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {

        void init();

        void update();

        void telemetry(Telemetry telemetry);
}
