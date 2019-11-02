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
public class CatHW_DriveBase extends CatHW_Subsystem
{
    /* Wheel measurements */
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // Accurate for a NeveRest Orbital 20
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     ODO_COUNTS_PER_REV        = 1440;     // 1440 for E4T from Andymark
    static final double     ODO_WHEEL_DIAMETER_INCHES = 2.0 ;     // For figuring circumference
    static final double     ODO_COUNTS_PER_INCH       = ODO_COUNTS_PER_REV / (ODO_WHEEL_DIAMETER_INCHES * 3.1415);

    /* Autonomous Drive Speeds */
    static final double     HYPER_SPEED             = 0.95;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     CHILL_SPEED             = 0.4;
    static final double     CREEP_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.6;


    ElapsedTime runTime = new ElapsedTime();
    double      timeout = 0;

    ColorSensor     leftColSen;
    ColorSensor     rightColSen;
    static boolean  isDone;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* LED stuff */
    public RevBlinkinLedDriver lights   = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    /* local OpMode members. */
    LinearOpMode opMode = null;
    /* Constructor */
    public CatHW_DriveBase(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Blinkin LED stuff //
        lights           = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern          = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

    }


    /**
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    // Driving Methods:
    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /**
         * Will scale down our power numbers if they are
         * greater than 1.0 so that we continue the set
         * course and don't just limit to the highest
         * possible value...
         */

        /**
         * Look at all motor values
         * Find the highest absolute value (the "scalor")
         * If the highest value is not more than 1.0, we don't need to change the values
         * But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
         * Finally, we pass out the scale factor so that we can scale each motor down
         */
        double scalor = 0;
        double scaleFactor;

        double[] values;
        values = new double[4];
        values[0] = Math.abs(leftFrontValue);
        values[1] = Math.abs(rightFrontValue);
        values[2] = Math.abs(leftBackValue);
        values[3] = Math.abs(rightBackValue);

        // Find highest value
        for(int i = 0; i+1 < values.length; i++){
            if(values[i] > scalor){
                scalor = values[i];
            }
        }

        // If the highest absolute value is over 1.0, we need to get to work!  Otherwise, we done here...
        if (scalor > 1.0) {
            // Get the reciprocal
            scaleFactor = 1.0 / scalor;
        } else {
            // Set to 1 so that we don't change anything we don'thave to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }

    /**
     * ---   ___________   ---
     * ---   IMU Methods   ---
     * ---   \/ \/ \/ \/   ---
     */
    public void IMUinit () {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        //the initialize method is taking a whole second
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
    }
    public int getCurrentAngle() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int)angles.firstAngle;
    }

    /**
     * ---   _____________   ---
     * ---   isDone Method   ---
     * ---  \/ \/ \/ \/ \/   ---
     */
    /*@Override
    public boolean isDone() {
        boolean keepDriving = true;
        if ((runTime.seconds() > timeout)) {
            Log.d("catbot", "Timed out.");
            keepDriving = false;
        }
        switch (currentMethod){
            case vertical:
                // One drive mode that drives blindly straight
                if (currentMode == DRIVE_MODE.driveTilDistance) {

                    //  Exit the method once robot stops
                    if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                            !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {
                        keepDriving = false;
                    }
                    Log.d("catbot", String.format("DriveVert LF: %d, %d;  RF: %d, %d;  LB: %d, %d;  RB %d,%d",
                            leftFrontMotor.getTargetPosition(),leftFrontMotor.getCurrentPosition(),
                            rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(),
                            leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(),
                            rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition()));
                }

                // The other drive mode using color sensors to fine lines
                if (currentMode == DRIVE_MODE.findLine) {

                    // Once left side hits color, turn left side motors off
                    if (mainHW.findLine(baseDelta, leftColSen)) {
                        leftFrontMotor.setPower(0.0);
                        leftRearMotor.setPower(0.0);

                        if (rightFrontMotor.getPower() == 0.0) {
                            keepDriving = false;
                        }
                    }
                    // Once right side hits color, turn right side motors off
                    if (mainHW.findLine(baseDelta, rightColSen)) {
                        rightFrontMotor.setPower(0.0);
                        rightRearMotor.setPower(0.0);

                        if (leftFrontMotor.getPower() == 0.0) {
                            keepDriving = false;
                        }
                    }
                }
                break;

            case horizontal:
                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                        !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {

                    keepDriving = false;
                }
                Log.d("catbot", String.format("DriveHor LF: %d, %d, %.2f;  RF: %d, %d, %.2f;  LB: %d, %d, %.2f;  RB %d,%d, %.2f",
                        leftFrontMotor.getTargetPosition(),leftFrontMotor.getCurrentPosition(),  leftFrontMotor.getPower(),
                        rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(), rightFrontMotor.getPower(),
                        leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(), leftRearMotor.getPower(),
                        rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition(), rightRearMotor.getPower()));

                break;

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

            case strafe:

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



                lBackPower = rFrontPower;
                rBackPower = lFrontPower;

                //add turn here
                if (Math.abs((getTheta - strafeAngle)) < Math.abs((getTheta - (strafeAngle + 360)))) {
                    if ((getTheta - strafeAngle) < 0) {
                        //turn right
                        if (Math.abs(getTheta - strafeAngle) > 4) {
                            rFrontPower = rFrontPower - strafeTurnPower;
                            rBackPower = rBackPower - strafeTurnPower;
                            lFrontPower = lFrontPower + strafeTurnPower;
                            lBackPower = lBackPower + strafeTurnPower;
                        }
                    }
                    else {
                        //turn left
                        if (Math.abs(getTheta - strafeAngle) > 4) {
                            rFrontPower = rFrontPower + strafeTurnPower;
                            rBackPower = rBackPower + strafeTurnPower;
                            lFrontPower = lFrontPower - strafeTurnPower;
                            lBackPower = lBackPower - strafeTurnPower;
                        }
                        }
                }
                else {
                    if ((getTheta - (strafeAngle + 360)) < 0) {
                        //turn right
                        if (Math.abs(getTheta - (strafeAngle + 360)) > 4) {
                            rFrontPower = rFrontPower - strafeTurnPower;
                            rBackPower = rBackPower - strafeTurnPower;
                            lFrontPower = lFrontPower + strafeTurnPower;
                            lBackPower = lBackPower + strafeTurnPower;
                        }
                    }
                    else {
                        //turn left
                            if (Math.abs(getTheta - (strafeAngle + 360)) > 4) {
                                rFrontPower = rFrontPower + strafeTurnPower;
                                rBackPower = rBackPower + strafeTurnPower;
                                lFrontPower = lFrontPower - strafeTurnPower;
                                lBackPower = lBackPower - strafeTurnPower;
                            }
                    }
                }
                double scale = findScalor(lFrontPower, rFrontPower, lBackPower, rBackPower);




                leftFrontMotor.setPower(lFrontPower  * strafePower * scale);
                rightFrontMotor.setPower(rFrontPower * strafePower * scale);
                leftRearMotor.setPower(lBackPower    * strafePower * scale);
                rightRearMotor.setPower(rBackPower   * strafePower * scale);

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
*/
    /**
     * ---   __________________   ---
     * ---   End of Our Methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket