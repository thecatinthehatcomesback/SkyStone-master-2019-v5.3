package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/**
 * Test_Class.java
 *
 *
 * Trying to debug with println() calls...
 *
 *
 * @author Team #10273, The Cat in the Hat Comes Back.
 */
public class Test_Class {

    /* Declare OpMode members. */


    public static void main(String [] args) {
        /* Go! */

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        allPoints.add(new CurvePoint(0, 0, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(1, 1, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(2, 2, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(3, 3, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(2, 4, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(4, 2, 3.0, 1.0, Math.toRadians(0), 1.0));

        followCurve(allPoints, 10.0, 0, 10.0, 90);
    }


    //----------------------------------------------------------------------------------------------
    // Pure Pursuit Methods:
    //----------------------------------------------------------------------------------------------

    private static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
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
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        // Quadratics Stuff
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = (Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0*m1*x1*y1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            // Do math for quadratic formula:
            double xRoot1 = (-quadraticB + (Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))))
                    / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            // Do math for other side of quadratic formula
            double xRoot2 = (-quadraticB - (Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))))
                    / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;


            // Add back the offset of the robot/circle's center
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            // Add back the offset to the other set of X and Y roots.
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;


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
    public static void followCurve(ArrayList<CurvePoint> allPoints, double maxPower, double followAngle, double turnSpeed, double theta) {
        //TODO:  Add some debug logs here...

        CurvePoint followThisPoint = getFollowPointPath(allPoints,
                new Point(0, 0),
                allPoints.get(0).followDistance, theta);

        //TODO:  Going to want to want to think about how we use the followAngle here...
        //goToPosition(followThisPoint.x, followThisPoint.y, followAngle);
        //translateDrive(followThisPoint.x, followThisPoint.y, maxPower, followAngle, turnSpeed, 5.0);

        //TODO:  Add logic to stop at the final point in the array list.
    }
    private static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation,
                                                 double followRadius, double theta) {
        // TODO: In case robot's follow radius doesn't intersect line...  Improve this later...  Use
        //  a line perpendicular perhaps?
        CurvePoint followThisPoint = new CurvePoint(pathPoints.get(0));

        // Go through all the CurvePoints and stop one early since a line needs at least two points.
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);
            System.out.println(i + "-startLine:   X=" + startLine.x +
                    "   Y=" + startLine.y);
            System.out.println(i + "-endLine:   X=" + endLine.x +
                    "   Y=" + endLine.y);


            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius,
                    startLine.toPoint(), endLine.toPoint());

            // Choose point that the robot is facing.
            double closestAngle = 1000000;

            // Set the robot to follow the point ahead of it/closest to its current heading angle.
            for (Point thisIntersection : intersections) {
                //TODO: Make sure this is all in Rads.
                double angle = Math.atan2(thisIntersection.y - robotLocation.y,
                        thisIntersection.x - robotLocation.x);
                double deltaAngle = Math.abs(angle - Math.toRadians(theta));
                System.out.println("angle = " + Math.toDegrees(angle));
                System.out.println("deltaAngle = " + Math.toDegrees(deltaAngle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followThisPoint.setPoint(thisIntersection);
                    System.out.println("closestAngle = " + Math.toDegrees(closestAngle));
                    System.out.println("setPoint(thisIntersection):   X=" + thisIntersection.x +
                            "   Y=" + thisIntersection.y);
                }
            }
            System.out.println();
        }
        System.out.println("followThisPoint:  " + followThisPoint.x + "   " + followThisPoint.y);
        return followThisPoint;
    }
}
