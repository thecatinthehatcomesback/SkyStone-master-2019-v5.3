/*
        CatHW_DriveClassic.java

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
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This is NOT an OpMode.
 *
 * This class is used when using the motor's encoders for autonomous.
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
public class CatHW_DriveClassic extends CatHW_DriveBase
{
    /* Enums */
    enum DRIVE_METHOD {
        vertical,
        horizontal,
        turn
    }

    enum DRIVE_MODE {
        findLine,
        followWall,
        driveTilDistance,
        driveUsingGyroStraight
    }

    enum TURN_MODE {
        SPIN,
        TANK
    }

    private DRIVE_MODE currentMode;
    private DRIVE_METHOD currentMethod;


    /* local OpMode members. */
    LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_DriveClassic(CatHW_Async mainHardware){
        super(mainHardware);
    }

    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Calls DriveBase's init
        super.init();

        // Sets enums to a default value
        currentMode   = DRIVE_MODE.driveTilDistance;
        currentMethod = DRIVE_METHOD.vertical;
    }

    /**
     * ---   _______________________   ---
     * ---   Driving Chassis Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    public void mecDriveVertical(double power, double distance, double timeoutS)  throws InterruptedException {
        mecDriveVertical(power, distance, timeoutS, DRIVE_MODE.driveTilDistance, null, null);
    }
    public void mecDriveVertical(double power, double distance, double timeoutS,
                                 DRIVE_MODE driveMode, ColorSensor leftColSenIn, ColorSensor rightColSenIn)  throws InterruptedException {
        /**
         * This is a simpler mecanum setDrivePowers method that drives blindly
         * straight vertically or using the color sensors to find a
         * line.
         */

        // Log message:
        Log.d("catbot", String.format(" Started setDrivePowers vert pow: %.2f, dist: %.2f, time:%.2f ", power, distance, timeoutS));


        currentMethod = DRIVE_METHOD.vertical;
        currentMode = driveMode;
        timeout = timeoutS;
        leftColSen = leftColSenIn;
        rightColSen = rightColSenIn;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        baseDelta = 0;
        isDone = false;
        if (driveMode == DRIVE_MODE.findLine) {
            // Turn on the color sensors we want and find the base alpha
            baseDelta = mainHW.findBaseDelta(rightColSen);
        }

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget  = (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget  = (int) (distance * COUNTS_PER_INCH);

            // Set the motors to travel towards their desired targets
            resetDriveEncoders();
            setDriveRunToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going backwards
            if (distance < 0) {
                power = -power;
            }

            // Due to the differences in weight on each wheel, adjust powers accordingly
            setDrivePowers(power, power, power, power);
        }
    }
    public void mecDriveHorizontal(double power, double distance, double timeoutS) {
        /**
         * This is a simpler mecanum setDrivePowers method that drives blindly
         * straight horizontally (positive numbers should translate left)
         */

        // Log message:
        Log.d("catbot", String.format(" Started setDrivePowers horizontal pow: %.2f, dist: %.2f, time:%.2f ",power,distance, timeoutS));


        currentMethod = DRIVE_METHOD.horizontal;
        timeout = timeoutS;
        isDone = false;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // (Multiply by sqrt of 2 to compensate)
            newLeftFrontTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightBackTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));

            // Set the motors to travel towards their desired targets
            resetDriveEncoders();
            setDriveRunToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going right
            if (distance < 0) {
                power = -power;
            }

            // Due to the differences in weight on each wheel, adjust powers accordingly
            setDrivePowers(power, power, power, power);
        }
    }
    public void advMecDrive(double power, double vectorDistance,
                            double vectorAng, double timeoutS) throws InterruptedException {
        /**
         * In this mecanum setDrivePowers method, we are trying to have the robot
         * setDrivePowers at an angle while the face of the robot remains pointed
         * ahead.
         *
         *
         * / = back left and forward right motors
         * \ = back right and forward front motors
         *
         * We add the Sin of our angle to the Cos in order to get the
         * powers for \ side of motors while we subtract Sin from Cos
         * for the / side of motors.
         */

        double leftFrontMod  = Math.cos(Math.toRadians(vectorAng)) + Math.sin(Math.toRadians(vectorAng));
        double rightFrontMod = Math.cos(Math.toRadians(vectorAng)) - Math.sin(Math.toRadians(vectorAng));
        double leftBackMod   = Math.cos(Math.toRadians(vectorAng)) - Math.sin(Math.toRadians(vectorAng));
        double rightBackMod  = Math.cos(Math.toRadians(vectorAng)) + Math.sin(Math.toRadians(vectorAng));

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        boolean keepDriving = true;
        isDone = false;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position and multiply each one to adjust for variation of mec wheels
            newLeftFrontTarget  = (int) (vectorDistance * COUNTS_PER_INCH * leftFrontMod);
            newRightFrontTarget = (int) (vectorDistance * COUNTS_PER_INCH * rightFrontMod);
            newLeftBackTarget   = (int) (vectorDistance * COUNTS_PER_INCH * leftBackMod);
            newRightBackTarget  = (int) (vectorDistance * COUNTS_PER_INCH * rightBackMod);

            // Set the motors to travel towards their desired targets
            resetDriveEncoders();
            setDriveRunToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going backwards
            if (vectorDistance < 0) {
                power = -power;
            }

            // Calculate motor setDrivePowers powers after we decide direction
            double SF = findScalor(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);
            leftFrontMod  = leftFrontMod  * SF * power;
            rightFrontMod = rightFrontMod * SF * power;
            leftBackMod   = leftBackMod   * SF * power;
            rightBackMod  = rightBackMod  * SF * power;
            // Drive
            setDrivePowers(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);

            while (opMode.opModeIsActive() &&
                    (runTime.seconds() < timeoutS) &&
                    keepDriving) {

                // Find the current positions so that we can display it later
                int leftFrontPosition  = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition   = leftRearMotor.getCurrentPosition();
                int rightBackPosition  = rightRearMotor.getCurrentPosition();

                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() && !rightFrontMotor.isBusy() &&
                        !leftRearMotor.isBusy() && !rightRearMotor.isBusy()) {
                    keepDriving = false;
                }

                // Display it for the driver
                opMode.telemetry.addData("New Path",  "Running to :%7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                opMode.telemetry.addData("Current Path",  "Running at :%7d :%7d :%7d :%7d",
                        leftFrontPosition, rightFrontPosition, leftBackPosition, rightBackPosition);
                opMode.telemetry.addData("Power: ", "%.3f", power);
                opMode.telemetry.addData("Time: ","%.4f seconds", runTime.seconds());
                opMode.telemetry.update();
            }


            // Stop all motion
            setDrivePowers(0, 0, 0, 0);
        }
    }
    public void mecTurn(double power, int degrees, double timeoutS) throws InterruptedException {
        mecTurn(power, degrees, timeoutS, TURN_MODE.SPIN);
    }
    public void mecTurn(double power, int degrees, double timeoutS,
                        TURN_MODE turnMode) throws InterruptedException {
        /**
         * Turns counterclockwise with a negative Z angle
         * Turns clockwise with a positive Z angle
         */

        currentMethod = DRIVE_METHOD.turn;
        timeout = timeoutS;
        isDone = false;

        // Ensure that the opMode is still active
        if (mainHW.opMode.opModeIsActive()) {
            targetAngleZ  = degrees;
            clockwiseTurn = (getCurrentAngle() < targetAngleZ);

            // Don't use encoders.  We only use the gyro angle to turn
            setDriveRunWithoutEncoders();
            // reset the timeout time and start motion.
            runTime.reset();

            // Log message:
            Log.d("catbot", String.format("Start turn...  target %d, current %d  %s", targetAngleZ, getCurrentAngle(), clockwiseTurn ?"CW":"CCW"));


            // Change the power based on which angle we are turning to
            if (clockwiseTurn) {
                leftFrontMotor.setPower(power);
                leftRearMotor.setPower(power);
                if (turnMode == TURN_MODE.SPIN) {
                    rightFrontMotor.setPower(-power);
                    rightRearMotor.setPower(-power);
                } else {
                    rightFrontMotor.setPower(-power/3);
                    rightRearMotor.setPower(-power/3);
                }
            } else {
                if (turnMode == TURN_MODE.SPIN) {
                    leftFrontMotor.setPower(-power);
                    leftRearMotor.setPower(-power);
                } else {
                    leftFrontMotor.setPower(-power/3);
                    leftRearMotor.setPower(-power/3);
                }
                rightFrontMotor.setPower(power);
                rightRearMotor.setPower(power);
            }
        }
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
            // Log message:
            Log.d("catbot", "Timed out.");
            keepDriving = false;
        }
        switch (currentMethod){
            case vertical:
                // One setDrivePowers mode that drives blindly straight
                if (currentMode == DRIVE_MODE.driveTilDistance) {

                    //  Exit the method once robot stops
                    if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                            !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {
                        keepDriving = false;
                    }
                    // Log message:
                    Log.d("catbot", String.format("DriveVert LF: %d, %d;  RF: %d, %d;  LB: %d, %d;  RB %d,%d",
                            leftFrontMotor.getTargetPosition(),leftFrontMotor.getCurrentPosition(),
                            rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(),
                            leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(),
                            rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition()));
                }

                // The other setDrivePowers mode using color sensors to fine lines
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
                // Log message:
                Log.d("catbot", String.format("DriveHor LF: %d, %d, %.2f;  RF: %d, %d, %.2f;  LB: %d, %d, %.2f;  RB %d,%d, %.2f",
                        leftFrontMotor.getTargetPosition(),leftFrontMotor.getCurrentPosition(),  leftFrontMotor.getPower(),
                        rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(), rightFrontMotor.getPower(),
                        leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(), leftRearMotor.getPower(),
                        rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition(), rightRearMotor.getPower()));

                break;

            case turn:

                int zVal = getCurrentAngle();

                // Log message:
                Log.d("catbot", String.format("turn  target %d, current %d  %s", targetAngleZ, zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal <= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal >= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                break;
        }

        if (!keepDriving){
            // Stop all motion
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