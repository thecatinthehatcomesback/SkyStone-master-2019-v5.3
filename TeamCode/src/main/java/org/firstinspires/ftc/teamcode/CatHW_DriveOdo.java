/*
        CatHW_DriveClassic.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the drive train.  This is a modified
    or stripped down version of CatSingleOverallHW to run all of intake
    movements.  This file is used by the new autonomous OpModes to run
    multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the specific hardware for the robot to
 * allow for multiple operations during autonomous.  In this case, that robot is //todo Change this name
 * Jack from the Cat in the Hat Comes Back team during the 2018-2019 season.
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  drive motor:        "left_rear"  & "left_front"
 * Motor channel:  Right drive motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatHW_DriveOdo extends CatHW_DriveBase
{
    /* Wheel measurements */
    static final double     ODO_COUNTS_PER_REV        = 1440;     // 1440 for E4T from Andymark
    static final double     ODO_WHEEL_DIAMETER_INCHES = 2.0 ;     // For figuring circumference
    static final double     ODO_COUNTS_PER_INCH       = ODO_COUNTS_PER_REV / (ODO_WHEEL_DIAMETER_INCHES * 3.1415);


    double          targetX;
    double          targetY;
    double          strafePower;
    double          strafeAngle;
    double          strafeTurnPower;


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

    enum TURN_MODE {
        SPIN,
        TANK
    }


    private DRIVE_METHOD currentMethod;
    private DRIVE_MODE currentMode;

    /* Public OpMode members. */
    // Motors
    public DcMotor  leftFrontMotor  = null;
    public DcMotor  rightFrontMotor = null;
    public DcMotor  leftRearMotor   = null;
    public DcMotor  rightRearMotor  = null;
    public DcMotor  rightOdometry   = null;
    public DcMotor  leftOdometry    = null;
    public DcMotor  backOdometry    = null;

    CatOdoPositionUpdate globalPositionUpdate;


    /* local OpMode members. */
    LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_DriveOdo(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        leftRearMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor   = hwMap.dcMotor.get("right_rear_motor");
        leftOdometry     = hwMap.dcMotor.get("left_odometry");
        rightOdometry    = hwMap.dcMotor.get("right_odometry");
        backOdometry     = hwMap.dcMotor.get("back_odometry");


        // Define motor directions //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // Define motor zero power behavior //
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOdometry.setDirection(DcMotor.Direction.FORWARD);
        rightOdometry.setDirection(DcMotor.Direction.FORWARD);
        backOdometry.setDirection(DcMotor.Direction.FORWARD);


        // Set motor modes //
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to run at no power so that the robot doesn't move during init //
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);


        // Sets enums to a default value
        currentMode = DRIVE_MODE.driveTilPoint;
        currentMethod = DRIVE_METHOD.translate;

        // Odometry Setup
        globalPositionUpdate  = new CatOdoPositionUpdate(leftOdometry, rightOdometry, backOdometry, ODO_COUNTS_PER_INCH, 25);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }


    /**
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    /* Basic methods for setting all four drive motor powers and setModes: */
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack) {
        /**
         * Simply setting the powers of each motor in less characters
         */
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);

        Log.d("catbot", String.format("Drive Power  LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFront, rightFront, leftBack, rightBack));
    }
    public void resetEncoders(){

        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    // Driving Methods:
    public void translateDrive(double x, double y, double power, double angle, double turnSpeed, double timeoutS){

        currentMethod = DRIVE_METHOD.translate;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        strafeAngle = angle;
        strafeTurnPower = turnSpeed;


        // Reset timer once called
        runTime.reset();
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
            Log.d("catbot", "Timed out.");
            keepDriving = false;
        }
        switch (currentMethod) {
            case turn:

                int zVal = getCurrentAngle();

                Log.d("catbot", String.format("target %d, current %d  %s", -targetAngleZ, -zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal >= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal <= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                break;

            case translate:

                double getY = globalPositionUpdate.returnYInches();
                double getX = globalPositionUpdate.returnXInches();
                double getTheta = globalPositionUpdate.returnOrientation();

                // if is good to end
                if ((Math.abs(targetY - getY) < 2 && Math.abs(targetX - getX) < 2)  && ( Math.abs(getTheta - strafeAngle) < 5|| (Math.abs(getTheta - (strafeAngle + 360)) < 5))) {

                    keepDriving = false;
                }

                //calc angle  - globalPositionUpdate.returnOrientation()
                double ang = (Math.atan2(targetX - getX, targetY - getY));
                double ang2 = ang - Math.toRadians(getTheta);
                //double ang = 0.785398;

                double lFrontPower = (Math.cos(ang2)  + Math.sin(ang2));
                double rFrontPower = (Math.cos(ang2) - Math.sin(ang2));
                double lBackPower;
                double rBackPower;


/*
                if ( Math.abs(lFrontPower) > Math.abs(rFrontPower)){
                    //if lFrontPower is greater than right
                    lFrontPower = (1 / Math.abs(lFrontPower)) * lFrontPower;
                    rFrontPower = (1 / Math.abs(lFrontPower)) * rFrontPower;
                }
                else {
                    //if rFrontPower is greater than left
                    lFrontPower = (1 / Math.abs(rFrontPower)) * lFrontPower;
                    rFrontPower = (1 / Math.abs(rFrontPower)) * rFrontPower;
                }
 */
                lBackPower = rFrontPower;
                rBackPower = lFrontPower;

                //todo: Add turn here
                if (Math.abs((getTheta - strafeAngle)) < Math.abs((getTheta - (strafeAngle + 360)))) {
                    if ((getTheta - strafeAngle) < 0) {
                        // Turn right
                        if (Math.abs(getTheta - strafeAngle) > 4) {
                            rFrontPower = rFrontPower - strafeTurnPower;
                            rBackPower  = rBackPower  - strafeTurnPower;
                            lFrontPower = lFrontPower + strafeTurnPower;
                            lBackPower  = lBackPower  + strafeTurnPower;
                        }
                    }
                    else {
                        // Turn left
                        if (Math.abs(getTheta - strafeAngle) > 4) {
                            rFrontPower = rFrontPower + strafeTurnPower;
                            rBackPower  = rBackPower  + strafeTurnPower;
                            lFrontPower = lFrontPower - strafeTurnPower;
                            lBackPower  = lBackPower  - strafeTurnPower;
                        }
                    }
                }
                else {
                    if ((getTheta - (strafeAngle + 360)) < 0) {
                        // Turn right
                        if (Math.abs(getTheta - (strafeAngle + 360)) > 4) {
                            rFrontPower = rFrontPower - strafeTurnPower;
                            rBackPower  = rBackPower  - strafeTurnPower;
                            lFrontPower = lFrontPower + strafeTurnPower;
                            lBackPower  = lBackPower  + strafeTurnPower;
                        }
                    }
                    else {
                        // Turn left
                        if (Math.abs(getTheta - (strafeAngle + 360)) > 4) {
                            rFrontPower = rFrontPower + strafeTurnPower;
                            rBackPower  = rBackPower  + strafeTurnPower;
                            lFrontPower = lFrontPower - strafeTurnPower;
                            lBackPower  = lBackPower  - strafeTurnPower;
                        }
                    }
                }

                // Calculate scale factor and motor powers
                double SF = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);
                leftFrontMotor.setPower(lFrontPower  * strafePower * SF);
                rightFrontMotor.setPower(rFrontPower * strafePower * SF);
                leftRearMotor.setPower(lBackPower    * strafePower * SF);
                rightRearMotor.setPower(rBackPower   * strafePower * SF);

                Log.d("catbot", String.format("translate LF: %.2f;  RF: %.2f;  LR: %.2f;  RR: %.2f  ; targetX/Y: %.2f %.2f ; currentX/Y %.2f %.2f ; calc/calc2/current angle: %.1f %.1f %.1f",
                        leftFrontMotor.getPower(), rightFrontMotor.getPower(), leftRearMotor.getPower(), rightRearMotor.getPower(),
                        targetX, targetY, getX, getY, Math.toDegrees(ang), Math.toDegrees(ang2), getTheta));
                break;
        }

        if (!keepDriving){
            // Stop all motion;
            drive(0, 0, 0, 0);
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