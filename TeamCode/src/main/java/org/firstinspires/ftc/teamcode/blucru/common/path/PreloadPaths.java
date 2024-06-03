package org.firstinspires.ftc.teamcode.blucru.common.path;

import com.acmerobotics.roadrunner.path.PathBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class PreloadPaths {
    public static Path buildBackdropFarPath() {
        return new PIDPathBuilder()
                .build();
    }
}
