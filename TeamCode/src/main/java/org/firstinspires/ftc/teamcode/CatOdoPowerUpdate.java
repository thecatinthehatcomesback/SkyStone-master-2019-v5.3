package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Written by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPowerUpdate {

    private CatOdoPositionUpdate positionUpdate;

    private ElapsedTime powerTime = new ElapsedTime();

    // Variables:
    private double currentPower;
    private double minPower = 0.15;
    private double maxPower;

    static final private double rampUpTime       = 400;  // In milliseconds
    static final private double rampDownDistance = 10;

    ArrayList<CurvePoint> curvePointsList;

    double radius;


    /**
     * Constructor which also has access to the values and such in CatOdoPositionUpdate
     * @param inPositionUpdate Create an instance of the CatOdoPositionUpdate
     */
    CatOdoPowerUpdate(CatOdoPositionUpdate inPositionUpdate) {
        positionUpdate = inPositionUpdate;
    }

    public void powerBoost(double power){
        minPower = power;
    }
    public void powerNormal(){
        minPower = .15;
    }


    public void setTarget(ArrayList<CurvePoint> curvePointsListIn, double radiusIn, double power) {
        curvePointsList = curvePointsListIn;
        maxPower = power;
        currentPower = minPower;
        powerTime.reset();
        radius = radiusIn;
    }

    /**
     * Use this method to continually update the powers for the
     * @return The power based on our motion profiling equations
     */
    public double updatePower() {

        // Update the current position
        double currentX    = positionUpdate.returnXInches();
        double currentY    = positionUpdate.returnYInches();
        double currentTime = powerTime.milliseconds();

        // Distance left to target calculation
        CurvePoint pointOnLine = positionUpdate.getFollowPointPath(curvePointsList,currentX,currentY,radius);
        double distanceToTarget = positionUpdate.distToPathEnd(pointOnLine.x,pointOnLine.y,curvePointsList) + positionUpdate.distance(currentX,currentY,pointOnLine.x,pointOnLine.y);

        if (currentPower >= (maxPower * (distanceToTarget / rampDownDistance))) {
            // Ramp down if within the rampDownDistance
            if (currentPower > minPower) {
                currentPower = maxPower * (distanceToTarget / rampDownDistance);
            } else {
                currentPower = minPower;
            }
        } else {
            // Ramp up power
            if (currentPower < maxPower) {
                currentPower = maxPower * (currentTime / rampUpTime);

            } else {
                currentPower = maxPower;
            }
        }
        if(currentPower < minPower){
            currentPower = minPower;
        }
        if(currentPower > maxPower){
            currentPower = maxPower;
        }

        // Finally!  Give the power!
        return currentPower;
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
