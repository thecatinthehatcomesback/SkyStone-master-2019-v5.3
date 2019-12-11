/*
        CatHW_Tail.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the tail/stacker.  This is a modified and
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all specific hardware related to the robot's tail
 * or stone stacker which will also allow for multiple operations during autonomous.
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
public class CatHW_Tail extends CatHW_Subsystem
{

    /* Public OpMode members. */
    private static final double GRABBER_OPEN = 1.0;
    private static final double GRABBER_CLOSE = -1.0;


    // Motors:
    public DcMotor tailLift     = null;
    public DcMotor tailExtend   = null;

    public Servo grabberServo   = null;

    /* local OpMode members. */

    // Timers:
    ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Tail(CatHW_Async mainHardware){
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors and Servos//
        tailLift        = hwMap.dcMotor.get("tail_lift");
        tailExtend      = hwMap.dcMotor.get("tail_extend");
        grabberServo    = hwMap.servo.get("grabber_servo");

        tailLift.setDirection(DcMotorSimple.Direction.REVERSE);
        tailExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Motor and Servo Modes //
        tailLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tailExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * ---   _______   ---
     * ---   Methods   ---
     * ---   \/ \/     ---
     */
    public void closeGrabber() {
        grabberServo.setPosition(GRABBER_CLOSE);
    }
    public void openGrabber() {
        grabberServo.setPosition(GRABBER_OPEN);
    }

    /* isDone stuff for CatHW_Jaws */
    static double TIMEOUT = 3.0;
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("left jaw power %.2f,", tailLift.getPower()));
        Log.d("catbot", String.format("right jaw power %.2f,", tailExtend.getPower()));
        return !(tailLift.isBusy() && tailExtend.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}// End of class bracket