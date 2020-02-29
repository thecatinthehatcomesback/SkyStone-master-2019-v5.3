package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Written by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPowerUpdate {

    private CatOdoPositionUpdate positionUpdate;

    private ElapsedTime powerTime = new ElapsedTime();

    // Variables:
    private double currentPower;
    private final double defaultMinPowerForward = 0.2;
    private final double minPowerStrafeScale = 0.75;
    private double minPower = defaultMinPowerForward;
    private double maxPower;
    private double distanceToTarget;

    static final private double rampUpTime       = 400;  // In milliseconds
    static final private double rampDownDistance = 23;

    private double targetX;
    private double targetY;


    /**
     * Constructor which also has access to the values and such in CatOdoPositionUpdate
     * @param inPositionUpdate Create an instance of the CatOdoPositionUpdate
     */
    CatOdoPowerUpdate(CatOdoPositionUpdate inPositionUpdate) {
        positionUpdate = inPositionUpdate;
    }

    public void reset(){
        minPower = defaultMinPowerForward;
    }

    public  void powerBoast(double power){
        minPower = power;
    }
    public  void powerNormal(){
        minPower = defaultMinPowerForward;
    }


    //if this one is called do not reset the timer  so it won't start the motors slowly
    public void setNonStopTarget(double x, double y, double power){
        targetX     = x;
        targetY     = y;
        maxPower    = power;
        currentPower = minPower;
        distanceToTarget = distance(positionUpdate.returnXInches(), positionUpdate.returnYInches(), targetX, targetY);

    }

    //the normal set target resets the timer so it will ramp up the power
    public void setTarget(double x, double y, double power) {
        powerTime.reset();
        setNonStopTarget(x,y,power);
    }
    public double getDistanceToTarget(){
        return distanceToTarget;
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
        distanceToTarget = distance(currentX, currentY, targetX, targetY);

        if (currentPower >= (1 * (distanceToTarget / rampDownDistance))) {
            // Ramp down if within the rampDownDistance
            currentPower = 1 * (distanceToTarget / rampDownDistance);

        } else {
            // Ramp up power
            currentPower = maxPower * (currentTime / rampUpTime);
        }

        if(currentPower < (minPower*calcMinPowerScale())){
            currentPower = minPower*calcMinPowerScale();
        }
        if(currentPower > maxPower){
            currentPower = maxPower;
        }

        // Finally!  Give the power!
        return currentPower;
    }

    public double calcMinPowerScale(){

        double minPowerScale;

        double absAngleToTarget         = (Math.atan2(targetX - positionUpdate.returnXInches(), targetY - positionUpdate.returnYInches()));
        double relativeAngleToTarget    = absAngleToTarget - Math.toRadians(positionUpdate.returnOrientation());

        //calculate a min power between 1 and 1.75 based off sin
        minPowerScale = (minPowerStrafeScale*(Math.abs(Math.sin(2*relativeAngleToTarget))))+1;

        //if it is between 45 and 135 set it to the strafe scale
        if (Math.abs(relativeAngleToTarget)% Math.PI > Math.PI/4 && Math.abs(relativeAngleToTarget)%Math.PI < 3*Math.PI/4){
         minPowerScale = 1 + minPowerStrafeScale;
        }
        return minPowerScale;
    }

    /**
     * Just a simple distance formula so that we know how long until robot reaches
     * the target with motion profiling.
     * @param currentX Enter in the positionUpdate.returnXInches()
     * @param currentY Enter in the positionUpdate.returnYInches()
     * @param targetX Set by the setTarget method inside the CatHW_DriveOdo.translateDrive
     * @param targetY Set by the setTarget method inside the CatHW_DriveOdo.translateDrive
     * @return distance
     */
    private double distance(double currentX, double currentY, double targetX, double targetY) {
        return Math.sqrt((targetX - currentX) *(targetX - currentX) + (targetY - currentY)*(targetY - currentY));
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
