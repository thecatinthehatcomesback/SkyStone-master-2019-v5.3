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
    double turnPower;


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
        rightOdometry    = hwMap.dcMotor.get("tail_lift2");
        backOdometry     = hwMap.dcMotor.get("right_jaw_motor");

        // Set odometry directions //
        //leftOdometry.setDirection(DcMotor.Direction.REVERSE);
        rightOdometry.setDirection(DcMotor.Direction.FORWARD);
       // backOdometry.setDirection(DcMotor.Direction.FORWARD);

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
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    /* Basic methods for setting all four setDrivePowers motor powers and setModes: */
    public void resetOdometryEncoders(){
        /**
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

                double minTP;
                minTP = (updatesThread.powerUpdate.getDistanceToTarget() - 10.0)/-30.0;

                if (minTP < 0){
                    minTP = 0;
                }
                if (minTP > .2){
                    minTP = .2;
                }

                turnPower = Math.abs((getTheta - targetTheta)/120.0);

                if (turnPower < minTP){
                    turnPower = minTP;
                }
                if (turnPower > .5){
                    turnPower = .5;
                }
                Log.d("catbot",  String.format("minTP: %.2f , TP: %.2f",minTP,turnPower));

                // Calculate scale factor and motor powers
                double SF = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                lFrontPower = lFrontPower  * getPower * SF;
                rFrontPower = rFrontPower  * getPower * SF;
                lBackPower = lBackPower  * getPower * SF;
                rBackPower = rBackPower  * getPower * SF;

                //adds turn
                if ((getTheta - targetTheta) < 0) {
                    // Turn right
                        rFrontPower = rFrontPower - (turnPower);
                        rBackPower = rBackPower - (turnPower);
                        lFrontPower = lFrontPower + (turnPower);
                        lBackPower = lBackPower + (turnPower);
                } else {
                    // Turn left
                        rFrontPower = rFrontPower + (turnPower);
                        rBackPower = rBackPower + (turnPower);
                        lFrontPower = lFrontPower - (turnPower);
                        lBackPower = lBackPower - (turnPower);
                }

                // Calculate scale factor and motor powers
                double SF2 = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                leftFrontMotor.setPower(lFrontPower  * SF2);
                rightFrontMotor.setPower(rFrontPower * SF2);
                leftRearMotor.setPower(lBackPower    * SF2);
                rightRearMotor.setPower(rBackPower   * SF2);

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