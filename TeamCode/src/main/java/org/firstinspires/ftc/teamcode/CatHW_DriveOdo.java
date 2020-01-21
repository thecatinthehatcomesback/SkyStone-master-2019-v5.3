/*
        CatHW_DriveOdo.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the setDrivePowers train.  This is a
    modified or stripped down version of CatSingleOverallHW to run all
    the drive train overall.  This file is used by the new autonomous
    OpModes to run multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * This is NOT an OpMode.
 *
 * This class is used when using the odometry encoders for autonomous.
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  setDrivePowers motor:        "left_rear"  & "left_front"
 * Motor channel:  Right setDrivePowers motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatHW_DriveOdo extends CatHW_DriveBase
{
    /* Wheel measurements */   //TODO:  Update these constants!
    static final double     ODO_COUNTS_PER_REV        = 8192;     // 8192 for rev encoder from rev robotics
    static final double     ODO_WHEEL_DIAMETER_INCHES = 2.0 ;     // For figuring circumference
    static final double     ODO_COUNTS_PER_INCH       = ODO_COUNTS_PER_REV / (ODO_WHEEL_DIAMETER_INCHES * Math.PI);


    double  targetX;
    double  targetY;
    double  strafePower;
    double  targetTheta;
    double  strafeTurnPower;


    /* Enums */
    enum DRIVE_METHOD {
        translate,
        turn
    }

    enum DRIVE_MODE {
        findLine,
        followWall,
        driveTilPoint
    }


    private DRIVE_MODE currentMode;
    private DRIVE_METHOD currentMethod;

    /* Public OpMode members. */
    // Motors
    public DcMotor  leftOdometry    = null;
    public DcMotor  rightOdometry   = null;
    public DcMotor  backOdometry    = null;

    CatOdoAllUpdates updatesThread;

    /* local OpMode members. */
    LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_DriveOdo(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Calls DriveBase's init
        super.init();


        // Define and Initialize Motors //
        //leftOdometry     = hwMap.dcMotor.get("left_jaw_motor");
        //rightOdometry    = hwMap.dcMotor.get("right_jaw_motor");
        //backOdometry     = hwMap.dcMotor.get("left_rear_motor");

        //leftOdometry     = hwMap.dcMotor.get("right_jaw_motor");
        //rightOdometry    = hwMap.dcMotor.get("left_rear_motor");
        //backOdometry     = hwMap.dcMotor.get("left_jaw_motor");

        leftOdometry     = hwMap.dcMotor.get("left_rear_motor");
        rightOdometry    = hwMap.dcMotor.get("left_jaw_motor");
        backOdometry     = hwMap.dcMotor.get("right_jaw_motor");

        // Set odometry directions //
        leftOdometry.setDirection(DcMotor.Direction.REVERSE);
        rightOdometry.setDirection(DcMotor.Direction.FORWARD);
        backOdometry.setDirection(DcMotor.Direction.FORWARD);

        // Set odometry modes //
        resetOdometryEncoders();

        // Odometry Setup
        updatesThread = new CatOdoAllUpdates(leftOdometry, rightOdometry, backOdometry, ODO_COUNTS_PER_INCH);
        Thread allUpdatesThread = new Thread(updatesThread);
        allUpdatesThread.start();

        // Sets enums to a default value
        currentMode = DRIVE_MODE.driveTilPoint;
        currentMethod = DRIVE_METHOD.translate;
    }

    /**
     * ---   ____________________   ---
     * ---   Pure Pursuit Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/   ---
     */
    public void goToPosition(double pointX, double pointY, double preferredAngle, double turnSpeed) {
        double distanceToPoint = Math.hypot(pointX - updatesThread.positionUpdate.returnXInches(),
                pointY - updatesThread.positionUpdate.returnYInches());

        double absAngleToPoint = Math.atan2(targetY - updatesThread.positionUpdate.returnYInches(),
                targetX - updatesThread.positionUpdate.returnXInches());

        double relativeAngleToPoint = absAngleToPoint - Math.toRadians(updatesThread.positionUpdate.returnOrientation());



        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToPoint;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToPoint;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        //TODO: Now, use all these numbers to move around (NB: They will always be limited to 1.0).


        double relativeTurnAngle = relativeAngleToPoint + preferredAngle;

        // This is mod used to limit the amount the robot turns.  The 30 is how fast it will turn...
        double turnMod = Range.clip(relativeTurnAngle / 30, -1, 1) * turnSpeed;
        // In the case that the target point is really close, robot won't turn
        if (distanceToPoint < 6) {
            turnMod = 0;
        }
    }
    public ArrayList<Double> lineCircleIntersection(double circleCenterX, double circleCenterY, double radius,
                                                    double linePoint1X, double linePoint1Y,
                                                    double linePoint2X, double linePoint2Y) {
        // Make sure we don't have a
        if (Math.abs(linePoint1X - linePoint2X) < 0.003) {
            linePoint1X = linePoint2X + 0.003;
        }
        if (Math.abs(linePoint1Y - linePoint2Y) < 0.003) {
            linePoint1Y = linePoint2Y + 0.003;
        }

        // Slope of line
        double m1 = (linePoint2Y - linePoint1Y) / (linePoint2X - linePoint1X);
        // Zeros around the robot/circle's center
        double x1 = linePoint1X - circleCenterX;
        double y1 = linePoint1Y - circleCenterY;

        // Quadratics Stuff
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = (Math.pow(m1, 2) * Math.pow(x1, 2)) - (2.0*m1*x1*y1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Double> allPoints = new ArrayList<>();

        try {
            // Do math for quadratic formula:
            double xRoot1 = (-quadraticB + (Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))))
                    / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            double xRoot2 = (-quadraticB - (Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))))
                    / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;


            // Add the offset of the robot/circle's center
            xRoot1 += circleCenterX;
            yRoot1 += circleCenterY;

            xRoot2 += circleCenterX;
            yRoot2 += circleCenterY;


            double minX = linePoint1X < linePoint2X ? linePoint1X : linePoint2X;
            double maxX = linePoint1X > linePoint2X ? linePoint1X : linePoint2X;

            if (xRoot1 > minX && xRoot1 < maxX) {
                //allPoints.add();  X1 Y1
            }
            if (xRoot2 > minX && xRoot2 < maxX) {
                //allPoints.add();  X2 Y2
            }
        } catch (Exception e) {

        }
        return allPoints;
    }

    /**
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    /* Basic methods for setting all four setDrivePowers motor powers and setModes: */
    public void resetOdometryEncoders(){
        /*
         * Set the odometry wheels to STOP_AND_RESET_ENCODER
         */
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    // Driving Methods:
    public void translateDrive(double x, double y, double power, double theta, double turnSpeed, double timeoutS){

        currentMethod = DRIVE_METHOD.translate;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        targetTheta = theta;
        strafeTurnPower = turnSpeed;

        // Reset timer once called
        runTime.reset();


        // Power update Thread:
        updatesThread.powerUpdate.setTarget(x, y, power);
    }
    public void quickDrive(double x, double y, double power, double theta, double turnSpeed, double timeoutS){

        translateDrive(x,y,power,theta,turnSpeed,timeoutS);
        waitUntilDone();
    }

    /**
     * ---   _____________   ---
     * ---   isDone Method   ---
     * ---  \/ \/ \/ \/ \/   ---
     */
    @Override
    public boolean isDone() {

        boolean keepDriving = true;


        if ((runTime.seconds() > timeout)) {
            Log.d("catbot", "Timed OUT.");
            keepDriving = false;
        }

        switch (currentMethod) {
            case turn:

                int zVal = getCurrentAngle();

                Log.d("catbot", String.format("target %d, current %d  %s",
                        -targetAngleZ, -zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                break;

            case translate:

                double getY = updatesThread.positionUpdate.returnYInches();
                double getX = updatesThread.positionUpdate.returnXInches();
                double getTheta = updatesThread.positionUpdate.returnOrientation();
                double getPower = updatesThread.powerUpdate.updatePower();

                // Check if ready to end
                if ((Math.abs(targetY - getY) < 2 && Math.abs(targetX - getX) < 2)  &&
                        (Math.abs(getTheta - targetTheta) < 5 )) {

                    keepDriving = false;
                }

                /**
                 * Calc robot angles:
                  */
                double absAngleToTarget         = (Math.atan2(targetX - getX, targetY - getY));
                double relativeAngleToTarget    = absAngleToTarget - Math.toRadians(getTheta);

                double lFrontPower = (Math.cos(relativeAngleToTarget) + Math.sin(relativeAngleToTarget));
                double rFrontPower = (Math.cos(relativeAngleToTarget) - Math.sin(relativeAngleToTarget));
                double lBackPower;
                double rBackPower;

                // Set powers for mecanum wheels
                lBackPower = rFrontPower;
                rBackPower = lFrontPower;

                //adds turn
                if ((getTheta - targetTheta) < 0) {
                    // Turn right
                    if (Math.abs(getTheta - targetTheta) > 4) {
                        rFrontPower = rFrontPower - (strafeTurnPower);
                        rBackPower = rBackPower - (strafeTurnPower);
                        lFrontPower = lFrontPower + (strafeTurnPower);
                        lBackPower = lBackPower + (strafeTurnPower);
                    }
                } else {
                    // Turn left
                    if (Math.abs(getTheta - targetTheta) > 4) {
                        rFrontPower = rFrontPower + (strafeTurnPower);
                        rBackPower = rBackPower + (strafeTurnPower);
                        lFrontPower = lFrontPower - (strafeTurnPower);
                        lBackPower = lBackPower - (strafeTurnPower);
                    }
                }


                // Calculate scale factor and motor powers
                double SF = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                leftFrontMotor.setPower(lFrontPower  * getPower * SF);
                rightFrontMotor.setPower(rFrontPower * getPower * SF);
                leftRearMotor.setPower(lBackPower    * getPower * SF);
                rightRearMotor.setPower(rBackPower   * getPower * SF);

                Log.d("catbot", String.format("translate LF: %.2f;  RF: %.2f;  LR: %.2f;  RR: %.2f  ; targetX/Y/Θ: %.2f %.2f %.1f; currentX/Y/Θ %.2f %.2f %.1f; pow %.2f",
                        leftFrontMotor.getPower(), rightFrontMotor.getPower(), leftRearMotor.getPower(), rightRearMotor.getPower(),
                        targetX, targetY, targetTheta, getX, getY, getTheta, getPower));
                break;
        }

        if (!keepDriving){
            // Stop all motion;
            setDrivePowers(0, 0, 0, 0);
            isDone = true;
            return true;
        }
        if (isDone){
            return true;
        }
        return false;
    }

    /**
     * ---   __________________   ---
     * ---   End of Our Methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket