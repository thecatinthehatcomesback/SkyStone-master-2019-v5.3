/*
        CatHW_Jaws.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the jaws.  This is a modified and
    stripped down version of CatSingleOverallHW to run all of jaws
    movements.  This file is used by the new autonomous OpModes to run
    multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all specific hardware related to the robot's jaws
 * or stone intake which will also allow for multiple operations during autonomous.
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
public class CatHW_Jaws extends CatHW_Subsystem
{

    /* Public OpMode members. */
    static final double JAW_POWER           = 0.9;


    // Motors:
   // public DcMotor leftJawMotor     = null;
    public DcMotor rightJawMotor    = null;

    /* local OpMode members. */

    // Timers:
    ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware){
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        //leftJawMotor    = hwMap.dcMotor.get("left_jaw_motor");
        rightJawMotor   = hwMap.dcMotor.get("right_jaw_motor");

        //leftJawMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightJawMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Motor and Servo Modes //
        //leftJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setJawPower(double power) {
        //leftJawMotor.setPower(power);
        rightJawMotor.setPower(power);
    }
    public void intakeJaws() {
        /**
         * Turn on both jaws motors to suck in:
         */

        //leftJawMotor.setPower(-JAW_POWER);
        rightJawMotor.setPower(JAW_POWER);
    }
    public void intakeJawsBlue() {
        /**
         * Turn on both jaws motors to suck in:
         has the left motor slightly faster to improve pick up chance on blue side
         */

        //leftJawMotor.setPower(-JAW_POWER );
        rightJawMotor.setPower(JAW_POWER - .15);
    }
    public void intakeJawsRed() {
        /**
         * Turn on both jaws motors to suck in:
         has the right motor slightly faster to improve pick up chance on red side
         */

        //leftJawMotor.setPower(-JAW_POWER  + .15);
        rightJawMotor.setPower(JAW_POWER);
    }

    public void outputJaws() {
        /**
         * Turn on both jaws motors to spit OUT:
         */

        //leftJawMotor.setPower(JAW_POWER*0.3);
        rightJawMotor.setPower(-JAW_POWER*0.3);
    }
    public void turnOffJaws() {
        /**
         * Turn off both jaws motors:
         */

        //leftJawMotor.setPower(0.0);
        rightJawMotor.setPower(0.0);
    }


    /* isDone stuff for CatHW_Jaws */
    static double TIMEOUT = 3.0;
    @Override
    public boolean isDone() {
        //Log.d("catbot", String.format("left jaw power %.2f,", leftJawMotor.getPower()));
        Log.d("catbot", String.format("right jaw power %.2f,", rightJawMotor.getPower()));
        return !( rightJawMotor.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}// End of class bracket