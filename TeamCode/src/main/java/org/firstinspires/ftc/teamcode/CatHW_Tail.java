/*
        CatHW_Tail.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and extension of the tail.  This is a modified and
    stripped  down version of CatSingleOverallHW to run all of jaws
    extending movements.  This file is used by the new autonomous OpModes
    to run multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the jaws specific hardware for the robot to
 * allow for multiple operations during autonomous.  In this case, that robot is
 * Jack from the Cat in the Hat Comes Back team during the 2018-2019 season.
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
    public Servo tailLeft   = null;
    public Servo tailRight  = null;


    /* local OpMode members. */
    LinearOpMode opMode     = null;

    /* Constructor */
    public CatHW_Tail(CatHW_Async mainHardware){

    super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {
       // tailLeft = hwMap.servo.get("left_tail");
       // tailRight = hwMap.servo.get("right_tail");
    }

    /**
     * ---   ____________   ---
     * ---   Tail Methods   ---
     * ---   \/ \/ \/ \/    ---
     */
    public void grabFoundationFingers() {
        tailLeft.setPosition(0.4);
        tailRight.setPosition(0.4);
    }
    public void releaseFoundationFingers() {
        tailLeft.setPosition(0);
        tailRight.setPosition(1.0);
    }


    @Override
    public boolean isDone() {

        return true;
    }


    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket