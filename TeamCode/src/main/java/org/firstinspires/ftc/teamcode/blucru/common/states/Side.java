package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum Side {
    CLOSE,
    FAR;

    public Side flip() {
        if(this == CLOSE) {
            return FAR;
        } else {
            return CLOSE;
        }
    }
}
