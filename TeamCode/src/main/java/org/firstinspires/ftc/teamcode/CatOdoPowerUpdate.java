package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Modified by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPowerUpdate {

    // Variables:
    public double currentPower  = 0.0;
    public double maxPower      = 0.0;
    public double minPower      = 0.0;

    public double currentLF     = 0.0;
    public double currentRF     = 0.0;
    public double currentLB     = 0.0;
    public double currentRB     = 0.0;

    public double previousTime      = 0.0;
    public double previousDistance  = 0.0;

    public double rampUPrate        = 0.0;
    public double rampDOWNrate      = 0.0;


    /**
     * Thoughts:
     *
     *
     * For ramp up:
     *
     * 1.  Needs a initial jump up (to say 0.2) to initially get over the static friction
     * 2.  Will then ramp up to max speed every time it wakes from the sleep period based
     * on the max power divided by time period left to get up to max speed to (0.5 sec e.g.)
     * 3.  Store this as currentPower then compare to max power (don't change anything if
     * current power is higher than maxPower)
     *
     *
     * For ramp down:
     *
     * 1.  Start ramping down as soon as within the distance it takes to slow down (e.g.
     * seven inches?)
     * 2.  Use the minPower
     *
     *
     * currentPower = currentPower * (deltaDistance * rate (which could be 1/7 or some
     * other calculation?)
     */
}
