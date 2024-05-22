package org.firstinspires.ftc.teamcode.blucru.common.util.wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class LimitSwitch {
    DigitalChannel digitalChannel;

    public LimitSwitch(DigitalChannel digitalChannel) {
        this.digitalChannel = digitalChannel;
    }

    public boolean isPressed() {
        return !digitalChannel.getState();
    }
}
