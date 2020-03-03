package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.util.ArrayList;

/**
 * CatOdoPositionUpdate.java
 *
 *
 * Class to keep track of robot's position using odometry modules.  Since our robot moves fast and
 * the data reading ends up not being consistent with the brief time lapse.  Thus, we added bulk
 * reads along with several different position equation changes and other work.
 *
 *
 * @author Original created by Sarthak (of Wizards.exe) on 6/1/2019.
 * @author Modified by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPositionUpdate
{
    // Odometry wheels:
    private DcMotor verticalEncoderLeft;
    private DcMotor verticalEncoderRight;
    private DcMotor horizontalEncoder;
    // Expansion hubs:
    private ExpansionHubEx expansionHub;

    // Position variables used for storage and calculations:
    private double verticalRightEncoderWheelPosition = 0;
    private double verticalLeftEncoderWheelPosition = 0;
    private double normalEncoderWheelPosition = 0;
    private double changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0;
    private double robotGlobalYCoordinatePosition = 0;
    private double robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0;
    private double previousVerticalLeftEncoderWheelPosition = 0;
    private double prevNormalEncoderWheelPosition = 0;

    // Algorithm constants:
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    // Files to access the algorithm constants:
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile(
            "wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile(
            "horizontalTickOffset.txt");

    private int verticalLeftEncoderPositionMultiplier = -1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = -1;
    private int leftEncoderValue = 0;
    private int rightEncoderValue = 0;
    private int horizontalEncoderValue = 0;
    private RevBulkData bulkData;
    private ElapsedTime time = new ElapsedTime();

    private double count_per_in;

    public boolean isUpdated = false;
    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry
     *                          encoder wheels
     */
    public CatOdoPositionUpdate(ExpansionHubEx inExpansionHub, DcMotor verticalEncoderLeft,
                                DcMotor verticalEncoderRight, DcMotor horizontalEncoder,
                                double COUNTS_PER_INCH) {
        count_per_in = COUNTS_PER_INCH;
        expansionHub = inExpansionHub;
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        time.reset();
    }

    public CatOdoPositionUpdate(ExpansionHubEx inExpansionHub, DcMotor verticalEncoderLeft,
                                DcMotor verticalEncoderRight, DcMotor horizontalEncoder,
                                double COUNTS_PER_INCH, double startingX, double startingY,
                                double startingOrientation) {
        count_per_in = COUNTS_PER_INCH;
        expansionHub = inExpansionHub;
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;

        this.robotGlobalXCoordinatePosition = startingX;
        this.robotGlobalYCoordinatePosition = startingY;
        this.robotOrientationRadians = Math.toRadians(startingOrientation);

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.
                readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.
                readFile(horizontalTickOffsetFile).trim());
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    public void globalCoordinatePositionUpdate() {
        // Do a bulk read of encoders:
        bulkData = expansionHub.getBulkInputData();
        leftEncoderValue = bulkData.getMotorCurrentPosition(verticalEncoderLeft);
        rightEncoderValue = bulkData.getMotorCurrentPosition(verticalEncoderRight);
        horizontalEncoderValue = bulkData.getMotorCurrentPosition(horizontalEncoder);
        // Get current positions:
        verticalLeftEncoderWheelPosition = (leftEncoderValue *
                verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (rightEncoderValue *
                verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition -
                previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition -
                previousVerticalRightEncoderWheelPosition;

        // Calculate angle:
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
        double robotOrientationRadiansHalf = robotOrientationRadians -
                (changeInRobotOrientation / 2);

        // Get the components of the motion:
        normalEncoderWheelPosition = (horizontalEncoderValue*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation *
                horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        double robotXChangeCounts = (p*Math.sin(robotOrientationRadiansHalf) + n*Math.cos(robotOrientationRadiansHalf));
        double robotYChangeCounts = (p*Math.cos(robotOrientationRadiansHalf) - n*Math.sin(robotOrientationRadiansHalf));
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + robotXChangeCounts;
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + robotYChangeCounts;

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;

        double velocityTimer = time.seconds();
        time.reset();

        double rotVelocity = (changeInRobotOrientation *(180/Math.PI))/velocityTimer;
        double velocity = (Math.sqrt(Math.pow(robotXChangeCounts,2) + Math.pow(robotYChangeCounts,2))/count_per_in)/velocityTimer;

        Log.d("catbot", String.format("OdoTicks L/R/B  :%7d  :%7d  :%7d   X: %5.2f  Y: %5.2f  theta: %5.2f Velocity: %5.2f RotVelocity: %5.2f",
                returnVerticalLeftEncoderPosition(),
                returnVerticalRightEncoderPosition(),
                returnNormalEncoderPosition(),
                returnXInches(),
                returnYInches(),
                returnOrientation(),
                velocity,
                rotVelocity));


        isUpdated = true;
    }

    public double returnXInches() { return robotGlobalXCoordinatePosition/count_per_in; }

    public double returnYInches() { return robotGlobalYCoordinatePosition/count_per_in; }


    public void resetPos() {
        verticalRightEncoderWheelPosition = 0;
        verticalLeftEncoderWheelPosition = 0;
        normalEncoderWheelPosition = 0;
        changeInRobotOrientation = 0;
        robotGlobalXCoordinatePosition = 0;
        robotGlobalYCoordinatePosition = 0;
        robotOrientationRadians = 0;
        previousVerticalRightEncoderWheelPosition = 0;
        previousVerticalLeftEncoderWheelPosition = 0;
        prevNormalEncoderWheelPosition = 0;
        leftEncoderValue = 0;
        rightEncoderValue = 0;
        horizontalEncoderValue = 0;

    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate() { return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate() { return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation() { return Math.toDegrees(robotOrientationRadians); }

    public int returnVerticalLeftEncoderPosition() {
        return (leftEncoderValue * verticalLeftEncoderPositionMultiplier);
    }

    public int returnVerticalRightEncoderPosition() {
        return (rightEncoderValue * verticalRightEncoderPositionMultiplier);
    }

    public int returnNormalEncoderPosition() {
        return (horizontalEncoderValue * normalEncoderPositionMultiplier);
    }

    public void reverseLeftEncoder() {
        if (verticalLeftEncoderPositionMultiplier == 1) {
            verticalLeftEncoderPositionMultiplier = -1;
         } else {
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder() {
        if (verticalRightEncoderPositionMultiplier == 1) {
            verticalRightEncoderPositionMultiplier = -1;
        } else {
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder() {
        if (normalEncoderPositionMultiplier == 1) {
            normalEncoderPositionMultiplier = -1;
        } else {
            normalEncoderPositionMultiplier = 1;
        }
    }

    public void setAngleOffset(double offset){
        robotOrientationRadians += Math.toRadians(offset);
    }

    public void setRobotGlobalCoordinate(double x, double y, double theta) {
        robotGlobalXCoordinatePosition = x;
        robotGlobalYCoordinatePosition = y;
        robotOrientationRadians = Math.toRadians(theta);
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


    // TODO: do this calc in the globalPosition update method, and just have a return target curve point method
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
            double smallestDist = Integer.MAX_VALUE;

            // Set the robot to follow the point ahead of it/closest to its current heading angle.
            for (Point thisIntersection : intersections) {

                double currentDist = distToPathEnd(thisIntersection.x,thisIntersection.y,pathPoints);
                System.out.println("current remaining distance = " + currentDist);

                if (currentDist < smallestDist){
                    smallestDist = currentDist;
                    System.out.println("smallest distance = " + smallestDist);
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







    /**
     * Just a simple distance formula so that we know how long until robot reaches
     * the target with motion profiling.
     * @param currentX Enter in the positionUpdate.returnXInches()
     * @param currentY Enter in the positionUpdate.returnYInches()
     * @param targetX Set by the setTarget method inside the CatHW_DriveOdo.translateDrive
     * @param targetY Set by the setTarget method inside the CatHW_DriveOdo.translateDrive
     * @return distance
     */
    public double distance(double currentX, double currentY, double targetX, double targetY) {
        return Math.hypot((targetX - currentX), (targetY - currentY));
    }

    public double distToPathEnd(double x, double y, ArrayList<CurvePoint> points) {
        //find what line the point is between
        int line = 0;
        for (int i = 0; i < points.size()-1; i++) {

            //if the the distance between the two points is the same as the distance EX: A-C------B
            if (distance(points.get(i).x, points.get(i).y, points.get(i + 1).x, points.get(i + 1).y) == distance(points.get(i).x, points.get(i).y, x, y) + distance(x, y, points.get(i + 1).x, points.get(i + 1).y)) {
                line = i;
            }
        }

        double totalDist = 0;
        //calc total dis by adding all distances
        for (int i = points.size()-1; i > line; i++) {
            totalDist += distance(points.get(i).x,points.get(i).y,points.get(i+1).x,points.get(i+1).y);

        }
        totalDist += distance(x,y,points.get(line + 1).x,points.get(line + 1).y);

        return totalDist;
    }


}