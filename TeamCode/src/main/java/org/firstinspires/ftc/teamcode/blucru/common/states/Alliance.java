package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum Alliance {
    RED(1),
    BLUE(-1);

    private int value;
    Alliance(int value) {
        this.value = value;
    }

    public Alliance flip() {
        value *= -1;
        return this;
    }
}
