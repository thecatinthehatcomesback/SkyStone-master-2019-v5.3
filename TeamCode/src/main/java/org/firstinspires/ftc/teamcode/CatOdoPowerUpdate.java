package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * CatOdoPowerUpdate.java
 *
 *
 * A class to calculate the powers for our robot's drive train using motion profiling.  This runs in
 * the separate updatesThread and works by getting information from the CatOdoPositionUpdate and
 * receiving information from other classes using the setTarget() setter method.  CatOdoPowerUpdate
 * will then take all that information into account, calculates according to our motion profiling
 * equations and returns a percentage at which the motors should be powered. TODO: Check again!!!
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatOdoPowerUpdate
{
    private CatOdoPositionUpdate positionUpdate;

    private ElapsedTime powerTime = new ElapsedTime();

    // Attributes/Fields: //
    private double currentPower;
    private final double defaultMinPowerForward = 0.2;
    private final double minPowerStrafeScale = 0.75;
    private double minPower = defaultMinPowerForward;
    private double maxPower;
    private double distanceToTarget;
    private CurvePoint pointOnLine;

    private static final double rampUpTime       = 400;  // In milliseconds
    private static final double rampDownDistance = 23;

    private ArrayList<CurvePoint> curvePointsList;

    private double followRadius;
    private int targetPoint;


    /**
     * Constructor which also has access to the values and such in CatOdoPositionUpdate
     * @param inPositionUpdate Create an instance of the CatOdoPositionUpdate
     */
    CatOdoPowerUpdate(CatOdoPositionUpdate inPositionUpdate) {
        positionUpdate = inPositionUpdate;
    }



    //----------------------------------------------------------------------------------------------
    // Setter and Getter Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Sets the robot's minimum power level.  TODO: this Javadoc
     *
     * @param power to set minPower to.
     */
    public void setMinPower(double power){
        minPower = power;
    }

    /**
     * Sets the robot's minimum power to the defaulted amount.
     */
    public void resetPowerToNormal(){
        minPower = defaultMinPowerForward;
    }


    /**
     * TODO:  if this one is called do not resetPowerTodefaultMin the timer  so it won't start the motors slowly
     *
     * @param power
     */
    public void setNonStopTarget(ArrayList<CurvePoint> pathPoints, double power) {

        maxPower = power;
        currentPower = minPower;
        curvePointsList = pathPoints;
        pointOnLine = getFollowPointPath(curvePointsList,
                positionUpdate.returnRobotPointInches(), followRadius);
        distanceToTarget = distToPathEnd(pointOnLine, curvePointsList) +
                distance(positionUpdate.returnRobotPointInches(), pointOnLine);
    }

    /**
     * Sets the target that the robot is going towards for this class.
     * TODO:  Check==>The normal set target resets the timer so it will ramp up the power
     *
     * @param power that the robot can go at most.
     */
    public void setTarget(ArrayList<CurvePoint> pathPoints, double power) {
        powerTime.reset();
        setNonStopTarget(pathPoints, power);
    }

    /**
     * @return distance from robot's current location to target's location in inches.
     */
    public double getDistanceToTarget(){
        return distanceToTarget;
    }



    //----------------------------------------------------------------------------------------------
    // Motion Profiling Stuff:
    //----------------------------------------------------------------------------------------------

    /**
     * Use this method to continually update the powers for the drive train.
     *
     * @return power based on our motion profiling equations.
     */
    public double updatePower() {

        // Update the current position:
        Point currentPos   = positionUpdate.returnRobotPointInches();
        double currentTime = powerTime.milliseconds();

        // Distance left to target calculation
        pointOnLine = getFollowPointPath(curvePointsList, currentPos , followRadius);
        distanceToTarget = distToPathEnd(pointOnLine,curvePointsList)
                + distance(currentPos, pointOnLine);

        if (currentPower >= (1 * (distanceToTarget / rampDownDistance))) {
            // Ramp down if within the rampDownDistance.
            currentPower = 1 * (distanceToTarget / rampDownDistance);

        } else {
            // Ramp up power.
            currentPower = maxPower * (currentTime / rampUpTime);
        }

        // Checks to make sure we are within our minimum and maximum power ranges.
        if(currentPower < (minPower*calcMinPowerScale())){
            currentPower = minPower*calcMinPowerScale();
        }
        if(currentPower > maxPower){
            currentPower = maxPower;
        }

        // Finally!  Give the power!
        return currentPower;
    }

    private double calcMinPowerScale(){

        double minPowerScale;

        double absAngleToTarget         = (Math.atan2(curvePointsList.get(targetPoint).x - positionUpdate.returnXInches(),
                curvePointsList.get(targetPoint).y - positionUpdate.returnYInches()));
        double relativeAngleToTarget    = absAngleToTarget - Math.toRadians(positionUpdate.returnOrientation());

        //calculate a min power between 1 and 1.75 based off sin
        minPowerScale = (minPowerStrafeScale*(Math.abs(Math.sin(2*relativeAngleToTarget))))+1;

        //if it is between 45 and 135 set it to the strafe scale
        if (Math.abs(relativeAngleToTarget)% Math.PI > Math.PI/4 && Math.abs(relativeAngleToTarget)%Math.PI < 3*Math.PI/4){
            minPowerScale = 1 + minPowerStrafeScale;
        }
        return minPowerScale;
    }


    //----------------------------------------------------------------------------------------------
    // Pure Pursuit Methods:
    //----------------------------------------------------------------------------------------------

    public ArrayList<Point> lineCircleIntersection(Point robotPos, double radius,
                                                   Point linePoint1, Point linePoint2) {
        // Make sure we don't have a slope of 1 or 0.
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }

        // Slope of line
        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        // Zeros around the robot/circle's center (remove offset of robot)
        double x1 = linePoint1.x - robotPos.x;
        double y1 = linePoint1.y - robotPos.y;

        // Quadratics Stuff
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = (Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0*m1*x1*y1) + Math.pow(y1, 2) -
                Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            // Do math for quadratic formula:
            double xRoot1 = (-quadraticB + (Math.sqrt(Math.pow(quadraticB, 2) -
                    (4.0 * quadraticA * quadraticC))))  /  (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            // Do math for other side of quadratic formula
            double xRoot2 = (-quadraticB - (Math.sqrt(Math.pow(quadraticB, 2) -
                    (4.0 * quadraticA * quadraticC))))  /  (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;


            // Add back the offset of the robot/circle's center
            xRoot1 += robotPos.x;
            yRoot1 += robotPos.y;
            // Add back the offset to the other set of X and Y roots.
            xRoot2 += robotPos.x;
            yRoot2 += robotPos.y;


            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            // Add point if the robot is on the first set of X and Y roots.
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }
            // Add point if the robot is on the second set of X and Y roots.
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception e) {
            // TODO:  Could throw an exception if taking sqrt of negative number...?
        }
        return allPoints;
    }


    // TODO: do this calc in the globalPosition update method, and just have a return target curve point method
    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints,
                                         Point robotLocation, double followRadius) {
        // TODO: In case robot's follow followRadius doesn't intersect line...  Improve this later...  Use
        //  a line perpendicular perhaps?
        CurvePoint followThisPoint = new CurvePoint(pathPoints.get(0));

        // Go through all the CurvePoints and stop one early since a line needs at least two points.
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);


            ArrayList<Point> intersections = lineCircleIntersection(robotLocation,
                    followRadius, startLine, endLine);

            // Choose point that the robot is facing.
            double smallestDist = Integer.MAX_VALUE;

            // Set the robot to follow the point ahead of it/closest to its current heading angle.
            for (Point thisIntersection : intersections) {

                double currentDist = distToPathEnd(thisIntersection.toCurvePoint(),pathPoints);

                if (currentDist < smallestDist){
                    smallestDist = currentDist;
                }
            }
        }
        return followThisPoint;
    }
    /*public void goToPosition(double pointX, double pointY, double preferredAngle) {
        double distanceToPoint = Math.hypot(pointX - updatesThread.positionUpdate.returnXInches(),
                pointY - updatesThread.positionUpdate.returnYInches());

        double absAngleToPoint = Math.atan2(targetY - updatesThread.positionUpdate.returnYInches(),
                targetX - updatesThread.positionUpdate.returnXInches());

        double relativeAngleToPoint = absAngleToPoint - Math.toRadians(updatesThread.positionUpdate.returnOrientation());



        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToPoint;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToPoint;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        // Now, use all these numbers to move around (NB: They will always be limited to 1.0).


        double relativeTurnAngle = relativeAngleToPoint + preferredAngle;

        // This is mod used to limit the amount the robot turns.  The 30 is how fast it will turn...
        double turnMod = Range.clip(relativeTurnAngle / 30, -1, 1);
        // In the case that the target point is really close, robot won't turn
        if (distanceToPoint < 6) {
            turnMod = 0;
        }
    }*/







    /**
     * Just a simple distance formula so that we know how long until robot reaches
     * the target with motion profiling.
     * @return distance
     */
    public double distance(Point point1, Point point2) {
        return Math.hypot((point2.x - point1.x), (point2.y - point1.y));
    }

    public double distToPathEnd(CurvePoint pointOnLine, ArrayList<CurvePoint> pathPoints) {

        int line = findLineNum(pointOnLine, pathPoints);

        double totalDist = 0;
        //calc total dis by adding all distances
        for (int i = pathPoints.size()-1; i > line; i++) {
            totalDist += distance(pathPoints.get(i), pathPoints.get(i+1));

        }
        totalDist += distance(pointOnLine, pathPoints.get(line + 1));

        return totalDist;
    }
    public int findLineNum(CurvePoint pointOnLine, ArrayList<CurvePoint> points){
        //find what line the point is between
        int line = 0;
        for (int i = 0; i < points.size()-1; i++) {

            //if the the distance between the two points is the same as the distance EX: A-C------B
            if (distance(points.get(i), points.get(i + 1))
                    == distance(points.get(i), pointOnLine)
                    + distance(pointOnLine, points.get(i + 1))) {
                line = i;
            }
        }
        //update the target point
        targetPoint = (line + 1);
        return line;

    }


}


    /*
    THOUGHTS:


    Goals and Plans:
    1.  First check to see if within ramp down range (if so, use scale down, otherwise
    start with the jump)
    2.  Begin ramping up power while checking the distance left
    3.  If ever within the distance in which we need to ramp down, ramp down.  Otherwise
    keep performing Step 4.
    4.  If currentPower is greater than maxPower, don't change currentPower.  Otherwise
    keep ramping up.

    For ramp UP:
    1.  Needs a initial jump up (to say 0.2 power) to initially get over the static friction
    2.  Will then ramp up to max speed every time it wakes from the sleep period based
    on the max power divided by time period left to get up to max speed to (0.5 sec e.g.)
    3.  Store this as currentPower then compare to max power (don't change anything if
    current power is higher than maxPower)

    For ramp DOWN:
    1.  Start ramping down as soon as within the distance it takes to slow down (e.g.
    seven inches?)
    2.  Use the minPower so that robot never stalls out until distance is reached.


    currentPower = currentPower * (deltaDistance * rate (which could be 1/7 or some
    other calculation?)
     */
