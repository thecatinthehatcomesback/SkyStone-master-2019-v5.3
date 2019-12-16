package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Written by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPowerUpdate {

    // Constants:
    static final private double RAMP_UP_TIME = 400;  // In milliseconds
    static final private double RAMP_DOWN_DISTANCE = 10;  // In inches
    // Variables:
    private double currentPower;
    private double minPower = 0.15;
    private double maxPower;
    private double targetX;
    private double targetY;
    // Constructors:
    private CatOdoPositionUpdate positionUpdate;
    // Timer:
    private ElapsedTime powerTime = new ElapsedTime();

    /**
     * Constructor which also has access to the values and such in CatOdoPositionUpdate.
     *
     * @param inPositionUpdate Create an instance of the CatOdoPositionUpdate.
     */
    CatOdoPowerUpdate(CatOdoPositionUpdate inPositionUpdate) {
        positionUpdate = inPositionUpdate;
    }

    public void powerNormal() {
        minPower = .15;
    }

    public void powerBoast(double power) {
        minPower = power;
    }

    /**
     * This method will set local variables in this class to the same values used in autonomous by
     * being called inside the CatHW_DriveOdo.translateDrive method.  This allows us to have the
     * information available in order to perform the motion profiling calculations.
     *
     * @param x     The new X coordinate the robot is traveling to.
     * @param y     The new X coordinate the robot is traveling to.
     * @param power Sets the new max power.
     */
    public void setTarget(double x, double y, double power) {
        targetX = x;
        targetY = y;
        maxPower = power;
        currentPower = minPower;
        powerTime.reset();
    }

    /**
     * Use this method to continually update the power calculation for our motion profiling.
     *
     * @return The power based on our motion profiling equations.
     */
    public double updatePower() {

        // Update the current position and time:
        double currentX = positionUpdate.returnXInches();
        double currentY = positionUpdate.returnYInches();
        double currentTime = powerTime.milliseconds();

        // Distance left to target calculation:
        double distanceToTarget = distance(currentX, currentY, targetX, targetY);

        if (currentPower >= (maxPower * (distanceToTarget / RAMP_DOWN_DISTANCE))) {
            // Ramp down if within the RAMP_DOWN_DISTANCE.
            if (currentPower > minPower) {
                currentPower = maxPower * (distanceToTarget / RAMP_DOWN_DISTANCE);
            } else {
                currentPower = minPower;
            }
        } else {
            // Ramp up power.
            if (currentPower < maxPower) {
                currentPower = maxPower * (currentTime / RAMP_UP_TIME);

            } else {
                currentPower = maxPower;
            }
        }

        // Make sure the power never goes above or below the min and max powers.
        if (currentPower < minPower) {
            currentPower = minPower;
        }
        if (currentPower > maxPower) {
            currentPower = maxPower;
        }

        // Finally!  Give the power!
        return currentPower;
    }

    /**
     * Just a simple distance formula so that we know how long until robot reaches the target with
     * motion profiling.
     *
     * @param currentX Enter in the positionUpdate.returnXInches().
     * @param currentY Enter in the positionUpdate.returnYInches().
     * @param targetX  Set by the setTarget method inside the CatHW_DriveOdo.translateDrive.
     * @param targetY  Set by the setTarget method inside the CatHW_DriveOdo.translateDrive.
     * @return distance left to target.
     */
    private double distance(double currentX, double currentY, double targetX, double targetY) {
        return Math.sqrt((targetX - currentX) * (targetX - currentX) + (targetY - currentY) * (targetY - currentY));
    }


    /**
     * Thoughts:
     *
     *
     * Plans:
     * 1.  First check to see if within ramp down range (if so, use scale down, otherwise
     * start with the jump)
     * 2.  Begin ramping up power while checking the distance left
     * 3.  If ever within the distance in which we need to ramp down, ramp down.  Otherwise
     * keep performing Step 4.
     * 4.  If currentPower is greater than maxPower, don't change currentPower.  Otherwise
     * keep ramping up.
     *
     * For ramp UP:
     * 1.  Needs a initial jump up (to say 0.2 power) to initially get over the static friction
     * 2.  Will then ramp up to max speed every time it wakes from the sleep period based
     * on the max power divided by time period left to get up to max speed to (0.5 sec e.g.)
     * 3.  Store this as currentPower then compare to max power (don't change anything if
     * current power is higher than maxPower)
     *
     * For ramp DOWN:
     * 1.  Start ramping down as soon as within the distance it takes to slow down (e.g.
     * seven inches?)
     * 2.  Use the minPower so that robot never stalls out until distance is reached.
     *
     *
     *
     * currentPower = currentPower * (deltaDistance * rate (which could be 1/7 or some
     * other calculation?))
     */
}
