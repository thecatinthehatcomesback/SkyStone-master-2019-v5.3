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
    static final double     ODO_COUNTS_PER_REV        = 1440;     // 1440 for E4T from Andymark
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
        leftOdometry     = hwMap.dcMotor.get("left_jaw_motor");
        rightOdometry    = hwMap.dcMotor.get("right_jaw_motor");
        backOdometry     = hwMap.dcMotor.get("left_rear_motor");

        // Set odometry directions //
        leftOdometry.setDirection(DcMotor.Direction.REVERSE);
        rightOdometry.setDirection(DcMotor.Direction.REVERSE);
        backOdometry.setDirection(DcMotor.Direction.REVERSE);

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
        strafeTurnPower = turnSpeed;

        // Reset timer once called
        runTime.reset();


        // Power update Thread:
        updatesThread.powerUpdate.setTarget(x, y, power);
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
                        (Math.abs(getTheta - strafeAngleEndTarget) < 5 )) {

                    keepDriving = false;
                }

                /**
                 * Calc robot angles:
                 *
                 *
                 * ang1 is the calculation of the angle
                 *
                 * ang2 is the 0 of the target angle with the current needed angles
                  */
                double ang1 = (Math.atan2(targetX - getX, targetY - getY));
                double ang2 = ang1 - Math.toRadians(getTheta);

                double lFrontPower = (Math.cos(ang2) + Math.sin(ang2));
                double rFrontPower = (Math.cos(ang2) - Math.sin(ang2));
                double lBackPower;
                double rBackPower;

                // Set powers for mecanum wheels
                lBackPower = rFrontPower;
                rBackPower = lFrontPower;

                //adds turn
                if ((getTheta - strafeAngleEndTarget) < 0) {
                    // Turn right
                    if (Math.abs(getTheta - strafeAngleEndTarget) > 4) {
                        rFrontPower = rFrontPower - (strafeTurnPower);
                        rBackPower = rBackPower - (strafeTurnPower);
                        lFrontPower = lFrontPower + (strafeTurnPower);
                        lBackPower = lBackPower + (strafeTurnPower);
                    }
                } else {
                    // Turn left
                    if (Math.abs(getTheta - strafeAngleEndTarget) > 4) {
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

                Log.d("catbot", String.format("translate LF: %.2f;  RF: %.2f;  LR: %.2f;  RR: %.2f  ; targetX/Y: %.2f %.2f ; currentX/Y %.2f %.2f ; calc/calc2/current angle/target angle: %.1f %.1f %.1f %.1f",
                        leftFrontMotor.getPower(), rightFrontMotor.getPower(), leftRearMotor.getPower(), rightRearMotor.getPower(),
                        targetX, targetY, getX, getY, Math.toDegrees(ang1), Math.toDegrees(ang2), getTheta, strafeAngleEndTarget));
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