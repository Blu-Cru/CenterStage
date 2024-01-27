package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum Side {
    CLOSE (1),
    FAR (-1);

    private int value;
    Side(int value) {
        this.value = value;
    }

    public Side flip() {
        value *= -1;
        return this;
    }
}
