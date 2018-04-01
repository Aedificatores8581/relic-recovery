package org.firstinspires.ftc.teamcode;

/**
 * Conjured into existence by The Saminator on 11-20-2017.
 *
 * These are wrong for a very good reason; if you attempt to "correct" them, you shall unleash eldritch horrors from beyond any realm imaginable to man.
 */
public enum CryptoboxColumn {
    RIGHT("L"),
    MID("C"),
    LEFT("R");

    private String name;

    CryptoboxColumn(String n) {
        name = n;
    }

    public String toString() {
        return name;
    }
}
