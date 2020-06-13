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
public class CatOdo_PowerUpdate
{
    //----------------------------------------------------------------------------------------------
    // Attributes/Fields:
    //----------------------------------------------------------------------------------------------
    private CatOdo_PositionUpdate positionUpdate;

    private ElapsedTime powerTime = new ElapsedTime();

    private final double defaultMinPowerForward = 0.2;
    private final double minPowerStrafeScale = 0.75;
    private double currentPower;
    private double maxPower;
    private double minPower = defaultMinPowerForward;

    /**
     * A constant that decides how much time it takes for the robot to reach its max power measured
     * in milliseconds.
     */
    private static final double rampUpTime = 400;
    /**
     * A constant which will be how far the robot needs to be in order to start slowing down
     * measured in inches.
     */
    private static final double rampDownDistance = 23.0;

    private double followRadius = 20.0;
    private double distanceToFinalTargetPoint;
    /** Number to keep track of which point in the simplePath the robot is driving towards. */
    private int targetPoint = 1;
    private CurvePoint pointOnLine;
    private ArrayList<CurvePoint> simplePathPoints;


    /**
     * Constructor which also has access to the values and such in CatOdoPositionUpdate.
     *
     * @param inPositionUpdate Create an instance of the CatOdoPositionUpdate
     */
    CatOdo_PowerUpdate(CatOdo_PositionUpdate inPositionUpdate) {
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
     *
     * @param simplePathPoints
     * @param maxPower
     * @param followRadius
     */
    public void setNonStopTarget(ArrayList<CurvePoint> simplePathPoints, double maxPower,
                                 double followRadius) {

        this.maxPower = maxPower;
        currentPower = minPower;
        this.simplePathPoints = simplePathPoints;
        this.followRadius = followRadius;
        pointOnLine = getFollowPoint(positionUpdate.returnRobotPointInches(), followRadius);
        distanceToFinalTargetPoint = distToPathEnd(pointOnLine) +
                distanceBetween(positionUpdate.returnRobotPointInches(), pointOnLine);
    }

    /**
     * Sets the target that the robot is going towards for this class.
     * TODO:  Check==>The normal set target resets the timer so it will ramp up the power
     *
     * @param power that the robot can go at most.
     */
    public void setTarget(ArrayList<CurvePoint> simplePathPoints, double power, double followRadius) {
        powerTime.reset();
        setNonStopTarget(simplePathPoints, power, followRadius);
    }

    /**
     * @return distanceBetween from robot's current location to target's location in inches.
     */
    public double getDistanceToFinalTargetPoint(){
        return distanceToFinalTargetPoint;
    }

    /**
     * @return the point on the line that the robot should be targeting
     */
    public CurvePoint getPointOnLine(){
        return pointOnLine;
    }

    /**
     * @return the point that the robot is currently targeting of the ones that were imputed in the ArrayList of points
     */
    public int getTargetPoint(){
        return targetPoint;
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
        //update time that is used in calculations for the ramp up only
        double currentTime = powerTime.milliseconds();

        // Distance left to target calculation
        // these will also update the target point
        pointOnLine = getFollowPoint(currentPos, followRadius);
        distanceToFinalTargetPoint = distToPathEnd(pointOnLine)
                + distanceBetween(currentPos, pointOnLine);


        if (currentPower >= (1 * (distanceToFinalTargetPoint / rampDownDistance))) {
            // Ramp down if within the rampDownDistance.
            currentPower = 1 * (distanceToFinalTargetPoint / rampDownDistance);

        } else {
            // Ramp up power.
            currentPower = maxPower * (currentTime / rampUpTime);
        }

        // Checks to make sure we are within our minimum and maximum power ranges.
        double minPowerScale = calcMinPowerScale();

        if(currentPower < (minPower*minPowerScale)){
            currentPower = minPower*minPowerScale;
        }
        if(currentPower > maxPower){
            currentPower = maxPower;
        }

        // Finally!  Give the power!
        return currentPower;
    }

    /**
     *
     * @return A value that will scale the min power based on the angle it is driving at because
     * strafing and driving at 45 degrees is harder than going straight forward
     */
    private double calcMinPowerScale(){

        double minPowerScale;

        double absAngleToTarget         = (Math.atan2(simplePathPoints.get(targetPoint).x -
                        positionUpdate.returnXInches(), simplePathPoints.get(targetPoint).y -
                positionUpdate.returnYInches()));
        double relativeAngleToTarget    = absAngleToTarget -
                Math.toRadians(positionUpdate.returnOrientation());

        // Calculate a minimum power between 1 and 1.75 based off sin.
        minPowerScale = (minPowerStrafeScale*(Math.abs(Math.sin(2*relativeAngleToTarget)))) + 1.0;

        // If relative angle to target is between 45 and 135 degrees, set it to the strafe scale.
        if (Math.abs(relativeAngleToTarget)% Math.PI > Math.PI/4 &&
                Math.abs(relativeAngleToTarget)%Math.PI < 3*Math.PI/4) {
            minPowerScale = 1.0 + minPowerStrafeScale;
        }
        return minPowerScale;
    }



    //----------------------------------------------------------------------------------------------
    // Pure Pursuit Methods:
    //----------------------------------------------------------------------------------------------

    /**
     *
     * @param robotPos the center of the circle should always be where the robot is
     * @param followRadius the followRadius of the circle tested for intersection
     * @param linePoint1 the first point of the line that will be tested to see if it intersects the circle
     * @param linePoint2 the second point of the line that will be tested to see if it intersects the circle
     * @return the points where the line and circle intersect what will be between 0 and 2 points if there are no intersections an empty arrayList will be returned
     */
    private ArrayList<Point> findPathIntersections(Point robotPos, double followRadius,
                                                   Point linePoint1, Point linePoint2) {
        // Make sure we don't have a slope of 1 or 0.
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }

        // Slope of line.
        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        // Zeros around the robot/circle's center (remove offset of robot).
        double x1 = linePoint1.x - robotPos.x;
        double y1 = linePoint1.y - robotPos.y;

        // Quadratics stuff...
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = (Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0*m1*x1*y1) + Math.pow(y1, 2) -
                Math.pow(followRadius, 2);
        double quadSqRtTerm = Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC);

        // Make sure robot is withing bounding box of the intersections.
        double minX = Math.min(linePoint1.x, linePoint2.x);
        double maxX = Math.max(linePoint1.x, linePoint2.x);

        // List of Points that intersect with the circle.
        ArrayList<Point> allIntersectingPoints = new ArrayList<>();


        // Try/Catch for first intersection.
        try {
            // Do math for quadratic formula:
            double xRoot1 = (-quadraticB + (Math.sqrt(quadSqRtTerm)))  /  (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            // Add back the offset of the robot/circle's center
            xRoot1 += robotPos.x;
            yRoot1 += robotPos.y;

            // Add point if the robot is on the first set of X and Y roots.
            if (xRoot1 > minX && xRoot1 < maxX) {
                allIntersectingPoints.add(new Point(xRoot1, yRoot1));
            }
        } catch (Exception e) {
            //TODO:  Better exception handling?
        }

        // Try/Catch for second intersection.
        try {
            // Do math for other side of quadratic formula
            double xRoot2 = (-quadraticB - (Math.sqrt(quadSqRtTerm)))  /  (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            // Add back the offset to the other set of X and Y roots.
            xRoot2 += robotPos.x;
            yRoot2 += robotPos.y;

            // Add point if the robot is on the second set of X and Y roots.
            if (xRoot2 > minX && xRoot2 < maxX) {
                allIntersectingPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception e) {
            //TODO:  Better exception handling?
        }

        /*
        Add the point from the simplePathPoints to the list of intersections if it is within the
        distance of the followRadius.
         */
        if (distanceBetween(linePoint1, robotPos) < followRadius) {
            allIntersectingPoints.add(linePoint1);
        }
        if (distanceBetween(linePoint2, robotPos) < followRadius){
            allIntersectingPoints.add(linePoint2);
        }

        return allIntersectingPoints;
    }


    /**
     *
     * @param robotLocation
     * @param followRadius
     * @return
     */
    private CurvePoint getFollowPoint(/*ArrayList<CurvePoint> pathPoints,*/
            Point robotLocation, double followRadius) {
        // TODO: In case robot's follow followRadius doesn't intersect line...
        //  Improve this later...  Use a line perpendicular perhaps?
        CurvePoint followThisPoint = new CurvePoint(simplePathPoints.get(0));

        // Go through all the CurvePoints and stop one early since a line needs at least two points.
        for (int i = 0; i < (simplePathPoints.size() - 1); i++) {
            CurvePoint startLine = simplePathPoints.get(i);
            CurvePoint endLine = simplePathPoints.get(i + 1);


            ArrayList<Point> intersections = findPathIntersections(robotLocation,
                    followRadius, startLine, endLine);

            // Choose point that the robot is facing.
            double smallestDist = Integer.MAX_VALUE;

            // Set the robot to follow the point ahead of it/closest to its current heading angle.
            for (int j = 0; j < intersections.size(); j++) {

                double currentDist = distToPathEnd(intersections.get(j).toCurvePoint());

                if (currentDist < smallestDist) {
                    smallestDist = currentDist;
                    followThisPoint = intersections.get(j).toCurvePoint();
                }
            }
        }
        return followThisPoint;
    }

    /**
     * Just a simple distanceBetween formula so that we know how long until robot reaches
     * the target with motion profiling.
     * @return distanceBetween
     */
    private double distanceBetween(Point point1, Point point2) {
        return Math.hypot((point2.x - point1.x), (point2.y - point1.y));
    }

    /**
     *
     * @param pointOnLine
     * @return
     */
    private double distToPathEnd(Point pointOnLine) {

        int line = findLineNum(pointOnLine);

        double totalDist = 0;
        //calc total dis by adding all distances
        for (int i = (simplePathPoints.size() - 1); i > line; i--) {
            totalDist += distanceBetween(simplePathPoints.get(i), simplePathPoints.get(i-1));

        }
        totalDist += distanceBetween(pointOnLine, simplePathPoints.get(line + 1));

        return totalDist;
    }

    /**
     *
     * @param pointOnLine
     * @return
     */
    private int findLineNum(Point pointOnLine){
        // Find what line the target point is between.
        int line = 0;

        for (int i = 0; i < (simplePathPoints.size() - 1); i++) {

            // If the the distanceBetween between the two points is the same as the distanceBetween: EX: A-C-----B
            if (Math.abs(distanceBetween(simplePathPoints.get(i), simplePathPoints.get(i + 1))
                    - (distanceBetween(simplePathPoints.get(i), pointOnLine)
                    + distanceBetween(pointOnLine, simplePathPoints.get(i + 1)))) < 0.05) {
                line = i;
            }
        }

        // Update the next point in the user's ArrayList that the robot is headed towards.
        this.targetPoint = (line + 1);
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