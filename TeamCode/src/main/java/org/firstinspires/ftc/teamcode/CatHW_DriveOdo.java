package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubEx;

import java.util.ArrayList;

/**
 * CatHW_DriveOdo.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the drive train using odometry modules as position givers.  This file is used by the
 * new autonomous OpModes to run multiple operations at once with odometry.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * NOTE: All names are lower case and have underscores between words.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_DriveOdo extends CatHW_DriveBase
{
    //----------------------------------------------------------------------------------------------
    // Odometry Module Constants:                               TODO: Are these constants updated???
    //----------------------------------------------------------------------------------------------

    /**
     * The number of encoder ticks per one revolution of the odometry wheel.
     * 8192 ticks for a REV encoder from REV Robotics.
     */
    private static final double ODO_COUNTS_PER_REVOLUTION = 8192;
    /** The measurement of the odometry wheel diameter for use in calculating circumference. */
    private static final double ODO_WHEEL_DIAMETER_INCHES = 2.0;
    /**
     * The amount of odometry encoder ticks equal to movement of 1 inch.  Used for conversion in the
     * robot's positioning algorithms so that when a user inputs (X,Y) coordinates in inches, those
     * can be converted into encoder ticks.
     */
    static final double ODO_COUNTS_PER_INCH     = ODO_COUNTS_PER_REVOLUTION /
            (ODO_WHEEL_DIAMETER_INCHES * Math.PI);

    private final double DEFAULT_FOLLOW_RADIUS = 6.0;

    //TODO: Other attributes/field should get some Javadoc sometime...
    private double targetX;
    private double targetY;
    private double targetTheta;
    private double xMin;
    private double xMax;
    private double yMin;
    private double yMax;
    private double thetaMin;
    private double thetaMax;
    private double lastPower = 0;
    private double maxPower;
    private double strafePower;

    private boolean isNonStop;

    /** Enumerated type for the style of drive the robot will make. */
    private enum DRIVE_METHOD {
        TRANSLATE,
        TURN
    }
    /** Variable to keep track of the DRIVE_METHOD that's the current style of robot's driving. */
    private DRIVE_METHOD currentMethod;

    private ArrayList<CurvePoint> targetPoints;
    private double followRadius = DEFAULT_FOLLOW_RADIUS;



    //----------------------------------------------------------------------------------------------
    // Public OpMode Members
    //----------------------------------------------------------------------------------------------

    // Motors
    public DcMotor  leftOdometry    = null;
    public DcMotor  rightOdometry   = null;
    public DcMotor  backOdometry    = null;
    public ExpansionHubEx expansionHub = null;

    // Access to Update Thread
    CatOdoAllUpdates updatesThread;

    /* Constructor */
    public CatHW_DriveOdo(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /**
     * Initialize standard Hardware interfaces in the CatHW_DriveOdo hardware.
     *
     * @throws InterruptedException in case of error.
     */
    public void init()  throws InterruptedException  {

        // Calls DriveBase's init: //
        super.init();

        // Define and Initialize Motors and Expansion Hub: //
        leftOdometry     = hwMap.dcMotor.get("left_rear_motor");
        rightOdometry    = hwMap.dcMotor.get("tail_lift2");
        backOdometry     = hwMap.dcMotor.get("right_jaw_motor");
        expansionHub     = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        // Set odometry directions: //
        //leftOdometry.setDirection(DcMotor.Direction.REVERSE);
        rightOdometry.setDirection(DcMotor.Direction.FORWARD);
        // backOdometry.setDirection(DcMotor.Direction.FORWARD);

        // Set odometry modes: //
        resetOdometryEncoders();

        // Odometry Setup: //
        updatesThread = updatesThread.getInstanceAndInit(expansionHub, leftOdometry, rightOdometry,
                backOdometry, ODO_COUNTS_PER_INCH);
        Thread allUpdatesThread = new Thread(updatesThread);
        updatesThread.resetThreads();
        allUpdatesThread.start();

        // Sets enums to a default value: //
        currentMethod = DRIVE_METHOD.TRANSLATE;
    }



    //----------------------------------------------------------------------------------------------
    // Driving Chassis Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the odometry wheels to STOP_AND_RESET_ENCODER.
     */
    public void resetOdometryEncoders() {
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Calls the translateDrive() method and adds in a waitUntilDone() afterwards so that the
     * autonomous code is cleaner without having so many waitUntilDone() methods clogging up the
     * view.
     *TODO
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */

    /**
     * Used to move the robot across the field.  The robot can also TURN while moving along the path
     * in order for it to face a certain by the end of its path.  This method assumes the robot has
     * odometry modules which give an absolute position of the robot on the field.
     *
     * TODO:
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void translateDrive(ArrayList<CurvePoint> points, double power, double theta,
                               double radius, double timeoutS){

        currentMethod = DRIVE_METHOD.TRANSLATE;
        timeout = timeoutS;
        isDone = false;
        targetPoints = points;
        strafePower = power;
        targetTheta = theta;
        followRadius = radius;

        CurvePoint targetPoint = getFollowPointPath(targetPoints,
                updatesThread.positionUpdate.returnXInches(),
                updatesThread.positionUpdate.returnYInches(), followRadius);
        targetX = targetPoint.x;
        targetY = targetPoint.y;


// Power update Thread:
        if (isNonStop){
            //if the last drive was nonstop
            updatesThread.powerUpdate.setNonStopTarget(targetX, targetY, power);
        }else {
            //if the last drive was normal
            updatesThread.powerUpdate.setTarget(targetX, targetY, power);
        }

        //set it so the next one will be nonstop
        isNonStop = true;

        // Reset timer once called
        runTime.reset();
    }

    /**
     * Nonstop TRANSLATE.  TODO: Add Javadoc here.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param finishedXMin
     * @param finishedXMax
     * @param finishedYMin
     * @param finishedYMax
     * @param finishedThetaMin
     * @param finishedThetaMax
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void translateDrive(double x, double y, double power, double theta, double finishedXMin,
                               double finishedXMax, double finishedYMin, double finishedYMax,
                               double finishedThetaMin, double finishedThetaMax, double timeoutS){

        currentMethod = DRIVE_METHOD.TRANSLATE;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        targetTheta = theta;
        xMin = finishedXMin;
        xMax = finishedXMax;
        yMin = finishedYMin;
        yMax = finishedYMax;
        thetaMin = finishedThetaMin;
        thetaMax = finishedThetaMax;
        maxPower = power;

        isNonStop = false;
        //if the last drive was nonstop
        updatesThread.powerUpdate.setNonStopTarget(x, y, power);

        // Reset timer once called
        runTime.reset();
    }





    //----------------------------------------------------------------------------------------------
    // Pure Pursuit Methods:
    //----------------------------------------------------------------------------------------------

    public ArrayList<Point> lineCircleIntersection(double robotX, double robotY, double radius,
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
        double x1 = linePoint1.x - robotX;
        double y1 = linePoint1.y - robotY;

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
            xRoot1 += robotX;
            yRoot1 += robotY;
            // Add back the offset to the other set of X and Y roots.
            xRoot2 += robotX;
            yRoot2 += robotY;


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
    public void followCurve(ArrayList<CurvePoint> allPoints, double maxPower, double followAngle, double turnSpeed) {
        //TODO:  Add some debug logs here...

        CurvePoint followThisPoint = getFollowPointPath(allPoints,
                updatesThread.positionUpdate.returnXInches(),
                updatesThread.positionUpdate.returnYInches(), allPoints.get(0).followDistance);

        //TODO:  Going to want to want to think about how we use the followAngle here...
        //goToPosition(followThisPoint.x, followThisPoint.y, followAngle);
        //translateDrive(followThisPoint.x, followThisPoint.y, maxPower, followAngle, turnSpeed, 5.0);

        //TODO:  Add logic to stop at the final point in the array list.
    }
    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints,
                                         double robotLocationX, double robotLocationY,
                                         double followRadius) {
        // TODO: In case robot's follow radius doesn't intersect line...  Improve this later...  Use
        //  a line perpendicular perhaps?
        CurvePoint followThisPoint = new CurvePoint(pathPoints.get(0));

        // Go through all the CurvePoints and stop one early since a line needs at least two points.
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);
            System.out.println(i + "startLine:   X=" + startLine.x +
                    "   Y=" + startLine.y);
            System.out.println(i + "endLine):   X=" + endLine.x +
                    "   Y=" + endLine.y);


            ArrayList<Point> intersections = lineCircleIntersection(robotLocationX, robotLocationY,
                    followRadius, startLine.toPoint(), endLine.toPoint());

            // Choose point that the robot is facing.
            double closestAngle = Integer.MAX_VALUE;

            // Set the robot to follow the point ahead of it/closest to its current heading angle.
            for (Point thisIntersection : intersections) {
                //TODO: Make sure this is all in Rads.
                double angle = Math.atan2(thisIntersection.y - updatesThread.positionUpdate.returnYInches(),
                        thisIntersection.x - updatesThread.positionUpdate.returnXInches());
                double deltaAngle = Math.abs(angle - Math.toRadians(updatesThread.positionUpdate.returnOrientation()));
                System.out.println("angle = " + angle);
                System.out.println("deltaAngle = " + deltaAngle);

                //TODO:  Will need to come back and revisit this in order to decide a much better
                // way to pick the point to follow.
                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followThisPoint.setPoint(thisIntersection);
                    System.out.println("closestAngle = " + closestAngle);
                    System.out.println("setPoint(thisIntersection):   X=" + thisIntersection.x +
                            "   Y=" + thisIntersection.y);
                }
            }
            System.out.println();
        }
        System.out.println("followThisPoint" + followThisPoint + "   " + followThisPoint.x + "   " + followThisPoint.y);
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



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {

        //wait for the update
        while (!updatesThread.positionUpdate.isUpdated){
            mainHW.opMode.sleep(1);
        }
        //set the updated status back to false so it knows that we are now using current powers
        updatesThread.positionUpdate.isUpdated = false;

        boolean keepDriving = true;


        // Exit if timeout is hit.  Helpful for when the robot stalls/stalls out.
        if ((runTime.seconds() > timeout)) {
            Log.d("catbot", "Timed OUT.");
            keepDriving = false;
        }

        switch (currentMethod) {
            case TURN:
                // Current orientation from odometry modules:
                int zVal = getCurrentAngle();

                Log.d("catbot", String.format("target %d, current %d  %s",
                        -targetAngleZ, -zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                break;

            case TRANSLATE:
                // Current robot position and orientation from odometry modules:
                double getY = updatesThread.positionUpdate.returnYInches();
                double getX = updatesThread.positionUpdate.returnXInches();
                double getTheta = updatesThread.positionUpdate.returnOrientation();
                double getPower = updatesThread.powerUpdate.updatePower();

                // Assign the point to follow
                CurvePoint targetPoint = getFollowPointPath(targetPoints, getX, getY, followRadius);
                targetX = targetPoint.x;
                targetY = targetPoint.y;

                // Check if ready to end without the isNonStop.
                if (!isNonStop) {
                    if ((Math.abs(targetY - getY) < 2 && Math.abs(targetX - getX) < 2) &&
                            (Math.abs(getTheta - targetTheta) < 5)) {

                        keepDriving = false;
                    }
                } else {

                    // if isNonStop
                    if (xMin < getX && getX < xMax && yMin < getY && getY < yMax &&
                            thetaMin < getTheta && getTheta < thetaMax){

                        keepDriving = false;
                    }

                }

                if (lastPower > getPower){
                    getPower = maxPower;
                }


                lastPower = getPower;


                /*
                Calculate robot angles:
                 */
                double absAngleToTarget         = (Math.atan2(targetX - getX, targetY - getY));
                double relativeAngleToTarget    = absAngleToTarget - Math.toRadians(getTheta);
                /*
                Calculate robot mecanum wheel powers:
                 */
                double lFrontPower = (Math.cos(relativeAngleToTarget) +
                        Math.sin(relativeAngleToTarget));
                double rFrontPower = (Math.cos(relativeAngleToTarget) -
                        Math.sin(relativeAngleToTarget));
                double lBackPower;
                double rBackPower;

                // Set powers for mecanum wheels:
                lBackPower = rFrontPower;
                rBackPower = lFrontPower;

                double minTP = (updatesThread.powerUpdate.getDistanceToTarget() - 6.0) / -20.0;

                if (minTP > .2){
                    minTP = .2;
                } else if (minTP < 0){
                    minTP = 0;
                }

                if ((getTheta - targetTheta) < 2) {
                    minTP = 0;
                }

                double turnPower = Math.abs((getTheta - targetTheta) / 120.0);

                if (turnPower < minTP){
                    turnPower = minTP;
                }
                if (turnPower > .5){
                    turnPower = .5;
                }
                Log.d("catbot",  String.format("minTP: %.2f , TP: %.2f",minTP, turnPower));

                /*
                Calculate scale factor and motor powers:
                 */
                double SF = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                lFrontPower = lFrontPower  * getPower * SF;
                rFrontPower = rFrontPower  * getPower * SF;
                lBackPower  = lBackPower   * getPower * SF;
                rBackPower  = rBackPower   * getPower * SF;

                // Adds TURN powers to each mecanum wheel.
                if ((getTheta - targetTheta) < 0) {
                    // Turn right
                    rFrontPower = rFrontPower - (turnPower);
                    rBackPower  = rBackPower  - (turnPower);
                    lFrontPower = lFrontPower + (turnPower);
                    lBackPower  = lBackPower  + (turnPower);
                } else {
                    // Turn left
                    rFrontPower = rFrontPower + (turnPower);
                    rBackPower  = rBackPower  + (turnPower);
                    lFrontPower = lFrontPower - (turnPower);
                    lBackPower  = lBackPower  - (turnPower);
                }

                // Calculate scale factor and motor powers:
                double SF2 = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                leftFrontMotor.setPower(lFrontPower  * SF2);
                rightFrontMotor.setPower(rFrontPower * SF2);
                leftRearMotor.setPower(lBackPower    * SF2);
                rightRearMotor.setPower(rBackPower   * SF2);

                Log.d("catbot", String.format("Translate LF:%.2f; RF:%.2f; LR:%.2f; RR:%.2f;" +
                                "   TargetX/Y/Θ: %.2f %.2f %.1f;" +
                                "   CurrentX/Y/Θ: %.2f %.2f %.1f;  Power: %.2f",
                        leftFrontMotor.getPower(), rightFrontMotor.getPower(),
                        leftRearMotor.getPower(), rightRearMotor.getPower(),
                        targetX, targetY, targetTheta,
                        getX, getY, getTheta, getPower));
                break;
        }

        if (!keepDriving) {
            if (isNonStop){
                isDone = true;
                return true;
            } else {
                // Stop all motion;
                setDrivePowers(0, 0, 0, 0);
                isDone = true;
                return true;
            }
        }
        return isDone;
    }
}