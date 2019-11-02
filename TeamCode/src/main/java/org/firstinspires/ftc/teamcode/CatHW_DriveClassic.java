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

    private DRIVE_METHOD currentMethod;
    private DRIVE_MODE currentMode;

    /* Public OpMode members. */
    // Motors
    public DcMotor  leftFrontMotor  = null;
    public DcMotor  rightFrontMotor = null;
    public DcMotor  leftRearMotor   = null;
    public DcMotor  rightRearMotor  = null;


    /* local OpMode members. */
    LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_DriveClassic(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");
        leftRearMotor    = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor   = hwMap.dcMotor.get("right_rear_motor");


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

        // Set motor modes //
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runNoEncoders();


        // Set all motors to run at no power so that the robot doesn't move during init //
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);


        // Sets enums to a default value
        currentMode   = DRIVE_MODE.driveTilDistance;
        currentMethod = DRIVE_METHOD.vertical;
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
        /**
         * Set all motors to STOP_AND_RESET_ENCODER
         */
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){
        /**
         * Set all motors to RUN_USING_ENCODER
         */
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runNoEncoders(){
        /**
         * Set all motors to RUN_WITHOUT_ENCODER
         */
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runToPosition(){
        /**
         * Set all motors to RUN_TO_POSITION
         */
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Driving Methods:
    public void mecDriveVertical(double power,
                                 double distance,
                                 double timeoutS)  throws InterruptedException {
        mecDriveVertical(power, distance, timeoutS, DRIVE_MODE.driveTilDistance, null, null);
    }
        public void mecDriveVertical(double power,
                                     double distance,
                                     double timeoutS, DRIVE_MODE driveMode, ColorSensor leftColSenIn, ColorSensor rightColSenIn)  throws InterruptedException {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight vertically or using the color sensors to find a
         * line.
         */

        Log.d("catbot", String.format(" Started drive vert pow: %.2f, dist: %.2f, time:%.2f ", power, distance, timeoutS));
        currentMethod = DRIVE_METHOD.vertical;
        currentMode = driveMode;
        timeout = timeoutS;
        leftColSen = leftColSenIn;
        rightColSen = rightColSenIn;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        boolean keepDriving = true;
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
            resetEncoders();
            runToPosition();
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

            drive(power, power, power, power);

        }
    }
    public void mecDriveHorizontal(double power, double distance, double timeoutS) {
        /**
         * This is a simpler mecanum drive method that drives blindly
         * straight horizontally (positive numbers should translate left)
         */
        Log.d("catbot", String.format(" Started drive horizontal pow: %.2f, dist: %.2f, time:%.2f ",power,distance, timeoutS));

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
            resetEncoders();
            runToPosition();
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
            drive(power, power, power, power);
        }
    }
    public void advMecDrive(double power, double vectorDistance,
                            double vectorAng, double timeoutS) throws InterruptedException {
        /**
         * In this mecanum drive method, we are trying to have the robot
         * drive at an angle while the face of the robot remains pointed
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
            resetEncoders();
            runToPosition();
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

            // Calculate motor drive powers after we decide direction
            double SF = findScalor(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);
            leftFrontMod  = leftFrontMod  * SF * power;
            rightFrontMod = rightFrontMod * SF * power;
            leftBackMod   = leftBackMod   * SF * power;
            rightBackMod  = rightBackMod  * SF * power;
            // Drive
            drive(leftFrontMod, rightFrontMod, leftBackMod, rightBackMod);

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
            drive(0, 0, 0, 0);
        }
    }
    public void mecTurn(double power, int degrees, double timeoutS) throws InterruptedException {
        mecTurn(power, degrees, timeoutS, TURN_MODE.SPIN);
    }
    public void mecTurn(double power, int degrees, double timeoutS, TURN_MODE turnMode) throws InterruptedException {
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
            runNoEncoders();
            // reset the timeout time and start motion.
            runTime.reset();
            Log.d("catbot", String.format("Start turn...  target %d, current %d  %s", -targetAngleZ, -getCurrentAngle(), clockwiseTurn ?"CW":"CCW"));


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