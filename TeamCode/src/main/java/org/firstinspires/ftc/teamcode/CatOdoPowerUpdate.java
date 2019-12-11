package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Modified by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPowerUpdate {

    private CatOdoPositionUpdate positionUpdate;

    ElapsedTime powerTime = new ElapsedTime();

    // Variables:
    double currentPower;
    double minPower = 0.1;
    double maxPower;

    double startTime;
    double currentTime;
    double distanceToTarget;
    double rampUpTime       = 2000;  // In milliseconds
    double rampDownDistance = 7;

    private double rampUPrate;
    private double rampDOWNrate;

    double currentX;
    double currentY;
    double targetX;
    double targetY;
    double targetTheta;


    /**
     * Constructor which also has access to the values and such in CatOdoPositionUpdate
     * @param inPositionUpdate Create an instance of the CatOdoPositionUpdate
     */
    CatOdoPowerUpdate(CatOdoPositionUpdate inPositionUpdate) {
        positionUpdate = inPositionUpdate;
    }


    public void setTarget(double x, double y, double theta, double power) {
        targetX     = x;
        targetY     = y;
        targetTheta = theta;
        maxPower    = power;
        //startTime   = time;

        powerTime.reset();
    }

    public double updatePower() {

        // Always begin power with a good chunk to overcome static friction

        // Update the current position
        currentX    = positionUpdate.returnXInches();
        currentY    = positionUpdate.returnYInches();
        currentTime = powerTime.milliseconds();

        // Distances
        distanceToTarget = distance(currentX, currentY, targetX, targetY);


        if (distanceToTarget < rampDownDistance) {
            // Ramp down if within the rampDownDistance
            if (currentPower > minPower) {
                currentPower = maxPower * (distanceToTarget / rampDownDistance);
            } else {
                currentPower = minPower;
            }
        } else {
            // Ramp up power
            //TODO: add the first time to minimum power
            if (currentPower < maxPower) {
                //TODO: might not be right

                rampUPrate = maxPower * ((currentTime - startTime) / rampUpTime);

                currentPower = currentPower + rampUPrate;
            } else {
                currentPower = maxPower;
            }
        }

        // Finally!  Give the power!
        return currentPower;
    }

    private double distance(double currentX, double currentY, double targetX, double targetY) {
        return Math.sqrt((targetX - currentX)*(targetX - currentX) + (targetY - currentY)*(targetY - currentY));
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
