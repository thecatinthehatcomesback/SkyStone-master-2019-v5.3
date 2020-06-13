package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A "hardware" class used to define all specific hardware related to the robot's jaws or stone intake which will also
 * allow for multiple operations during autonomous.
 * <p>
 * This is NOT an OpMode.  This class is used in tandem with all the other hardware classes.  This hardware class
 * assumes the device names have been configured on the robot.
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem {

    //------------------------------------------------------------------------------------------------------------------
    // Attributes and Constants:
    //------------------------------------------------------------------------------------------------------------------

    /** Power for the spinning jaws that will intake and output the Stones. */
    private static final double JAW_POWER = 0.9;


    // Motors:
    private DcMotor jawMotors = null;

    // Timers:
    private ElapsedTime runtime = new ElapsedTime();



    //------------------------------------------------------------------------------------------------------------------
    // Setup Methods:
    //------------------------------------------------------------------------------------------------------------------

    /**
     * Constructor method that calls the constructor method (using the keyword 'super') of this class' parent class.
     *
     * @param mainHardware needs to be the owner hardware class (AKA the CatHW_Async class).
     */    public CatHW_Jaws(CatHW_Async mainHardware) {

        super(mainHardware);
    }


    /**
     * Initialize standard Hardware interfaces for the Jaw hardware.
     */
    public void init() {

        // Define and Initialize Motors:
        jawMotors = hwMap.dcMotor.get("right_jaw_motor");

        // Set Motor Direction and Mode:
        jawMotors.setDirection(DcMotorSimple.Direction.FORWARD);
        jawMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the power for the jaws.
     */
    public void setJawPower(double power) {

        jawMotors.setPower(power);
    }

    /**
     * Turns on the jaws and sets them to intake.
     */
    public void intakeJaws() {

        jawMotors.setPower(JAW_POWER);
    }

    /**
     * Turn on both jaws motors to suck in (has the left motor slightly faster to improve pick up chance on blue side).
     */
    public void intakeJawsBlue() {

        jawMotors.setPower(JAW_POWER - .15);
    }

    /**
     * Turn on both jaws motors to suck in (has the left motor slightly faster to improve pick up chance on red side).
     */
    public void intakeJawsRed() {

        jawMotors.setPower(JAW_POWER);
    }

    /**
     * Turn on both jaws motors to spit OUT.
     */
    public void outputJaws() {

        jawMotors.setPower(-JAW_POWER * 0.3);
    }

    /**
     * Turn off both jaw motors.
     */
    public void turnOffJaws() {

        jawMotors.setPower(0.0);
    }



    //------------------------------------------------------------------------------------------------------------------
    // isDone Method:
    //------------------------------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {

        Log.d("catbot", String.format("Jaw power %.2f,", jawMotors.getPower()));
        double TIMEOUT = 3.0;
        return !(jawMotors.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}