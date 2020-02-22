package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LightPattern {
    private int delayMs;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    public LightPattern(int delayMsIn, RevBlinkinLedDriver.BlinkinPattern patternIn){
        delayMs = delayMsIn;
        pattern = patternIn;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    public int getDelayMs() {
        return delayMs;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }
}
