package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.revextensions2.ExpansionHubEx;
//TODO:  Download this import statement.

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
    /* Wheel measurements */                //TODO:  Update these constants!
    // __8192__ for REV encoder from REV Robotics:
    private static final double ODO_COUNTS_PER_REV        = 8192;
    // For figuring circumference:
    private static final double ODO_WHEEL_DIAMETER_INCHES = 2.0 ;
    // For use in determining robot's position:
    static final double         ODO_COUNTS_PER_INCH       = ODO_COUNTS_PER_REV /
            (ODO_WHEEL_DIAMETER_INCHES * Math.PI);


    private double targetX;
    private double targetY;
    private double strafePower;
    private double targetTheta;
    private double xMin;
    private double xMax;
    private double yMin;
    private double yMax;
    private double thetaMin;
    private double thetaMax;
    private double lastPower = 0;
    private double maxPower;

    private boolean isNonStop;


    /* Enums */
    enum DRIVE_METHOD {
        translate,
        turn
    }

    private DRIVE_METHOD currentMethod;


    /* Public OpMode members. */
    // Motors
    public DcMotor  leftOdometry    = null;
    public DcMotor  rightOdometry   = null;
    public DcMotor  backOdometry    = null;
    public ExpansionHubEx expansionHub = null;

    CatOdoAllUpdates updatesThread;

    /* local OpMode members. */
    LinearOpMode opMode = null;

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

        // Set odometry directions //
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
        currentMethod = DRIVE_METHOD.translate;
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
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will turn to while driving.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void quickDrive(double x, double y, double power, double theta, double timeoutS){

        translateDrive(x,y,power,theta,timeoutS);
        waitUntilDone();
    }

    /**
     * Used to move the robot across the field.  The robot can also turn while moving along the path
     * in order for it to face a certain by the end of its path.  This method assumes the robot has
     * odometry modules which give an absolute position of the robot on the field.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will turn to while driving.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void translateDrive(double x, double y, double power, double theta, double timeoutS){

        currentMethod = DRIVE_METHOD.translate;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        targetTheta = theta;

        // Power update Thread:
        updatesThread.powerUpdate.setTarget(x, y, power);

        //if the last drive was nonstop
        if (isNonStop){
            isNonStop = false;
            updatesThread.powerUpdate.setTimer(runTime);
        }

        // Reset timer once called
        runTime.reset();
    }

    /**
     * Nonstop translate.  TODO: Add Javadoc here.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will turn to while driving.
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

        currentMethod = DRIVE_METHOD.translate;
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

        //if the last drive was nonstop
        if (isNonStop){
            updatesThread.powerUpdate.setTimer(runTime);
        }

        // Power update Thread:
        updatesThread.powerUpdate.setTarget(x, y, power);

        isNonStop = true;

        // Reset timer once called
        runTime.reset();

    }



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {

        // Begins true so that it doesn't immediately exit.
        boolean keepDriving = true;


        // Exit if timeout is hit.  Helpful for when the robot stalls/stalls out.
        if ((runTime.seconds() > timeout)) {
            Log.d("catbot", "Timed OUT.");
            keepDriving = false;
        }

        switch (currentMethod) {
            case turn:
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

            case translate:
                // Current robot position and orientation from odometry modules:
                double getY = updatesThread.positionUpdate.returnYInches();
                double getX = updatesThread.positionUpdate.returnXInches();
                double getTheta = updatesThread.positionUpdate.returnOrientation();
                double getPower = updatesThread.powerUpdate.updatePower();

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
                    if (lastPower > getPower){
                        getPower = maxPower;
                    }

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

                // Adds turn powers to each mecanum wheel.
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