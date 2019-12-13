/*
        CatHW_Claw.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and extension of the claw.  This is a modified and
    stripped  down version of CatSingleOverallHW to run all of the
    foundation claw movements.  This file is used by the new autonomous
    OpModes to run multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the claw specific hardware for the robot to
 * allow for multiple operations during autonomous.
 *
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  setDrivePowers motor:        "left_rear"  & "left_front"
 * Motor channel:  Right setDrivePowers motor:        "right_rear" & "right_front"
 * And so on...
 */

public class CatHW_Claw extends CatHW_Subsystem
{

    private static final double CLAW_UP     = -0.5;
    private static final double CLAW_DOWN   = 0.5;

    /* Public OpMode members. */
    public Servo rightFoundationClaw = null;
    public Servo leftFoundationClaw  = null;


    /* local OpMode members. */
    LinearOpMode opMode     = null;

    /* Constructor */
    public CatHW_Claw(CatHW_Async mainHardware){

    super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {
        rightFoundationClaw = hwMap.servo.get("right_claw_servo");
        leftFoundationClaw  = hwMap.servo.get("left_claw_servo");

        // Pull the claw in to fit within sizing cube:
        retractClaws();
    }

    /**
     * ---   ____________   ---
     * ---   Claw Methods   ---
     * ---   \/ \/ \/ \/    ---
     */
    public void extendClaws() {
        rightFoundationClaw.setPosition(0.46);
        leftFoundationClaw.setPosition(.18);
    }
    public void retractClaws() {
        //right starts at 1 and moves to .46 for a total movement of .54
        rightFoundationClaw.setPosition(1);
        //left starts at 0 and moves to .18 for a total movement of .18
        leftFoundationClaw.setPosition(0);
    }


    @Override
    public boolean isDone() {
        // There's nothing to do, so isDone() always true.
        return true;
    }

    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket