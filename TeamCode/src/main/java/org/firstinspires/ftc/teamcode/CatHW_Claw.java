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

    /* Public OpMode members. */
    public Servo foundationClaw = null;


    /* local OpMode members. */
    LinearOpMode opMode     = null;

    /* Constructor */
    public CatHW_Claw(CatHW_Async mainHardware){

    super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {
        foundationClaw = hwMap.servo.get("claw_servo");

        // Pull the claw in to fit within sizing cube:
        retractClaw();
    }

    /**
     * ---   ____________   ---
     * ---   Claw Methods   ---
     * ---   \/ \/ \/ \/    ---
     */
    public void extendClaw() {
        foundationClaw.setPosition(1.0);
    }
    public void retractClaw() {
        foundationClaw.setPosition(-1.0);
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