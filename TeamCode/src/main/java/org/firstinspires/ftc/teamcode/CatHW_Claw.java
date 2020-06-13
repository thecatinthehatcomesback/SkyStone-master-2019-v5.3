package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * A "hardware" class containing all common code for accessing hardware specific to the movement and extension of the
 * foundation claws.  This file is used by the asynchronous autonomous OpModes to run multiple robot operations at
 * once.
 * <p>
 * This is NOT an OpMode.  This class is used in tandem with all the other hardware classes.  This hardware class
 * assumes the device names have been configured on the robot.
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Claw extends CatHW_Subsystem {

    //------------------------------------------------------------------------------------------------------------------
    // Attributes:
    //------------------------------------------------------------------------------------------------------------------

    private Servo rightFoundationClaw = null;
    private Servo leftFoundationClaw  = null;
    private Servo capstoneClaw        = null;



    //------------------------------------------------------------------------------------------------------------------
    // Setup Methods:
    //------------------------------------------------------------------------------------------------------------------

    /**
     * Constructor method that calls the constructor method (using the keyword 'super') of this class' parent class.
     *
     * @param mainHardware needs to be the owner hardware class (AKA the CatHW_Async class).
     */
    public CatHW_Claw(CatHW_Async mainHardware) {

        super(mainHardware);
    }

    /**
     * Initialize standard Hardware interfaces for the Claw subsystem.
     */
    public void init() {

        rightFoundationClaw = hwMap.servo.get("right_claw_servo");
        leftFoundationClaw = hwMap.servo.get("left_claw_servo");
        capstoneClaw = hwMap.servo.get("capstone_servo");

        // Pull the claw in to fit within sizing cube:
        retractClaws();
        grabCapstone();
    }



    //------------------------------------------------------------------------------------------------------------------
    // Claw Methods:
    //------------------------------------------------------------------------------------------------------------------

    /**
     * Bring the foundation claws down and latch onto the foundation.
     */
    public void extendClaws() {
        // Right starts at 1 and moves to .40 for a total movement of .57
        rightFoundationClaw.setPosition(0.43);
        // Left starts at 0 and moves to .18 for a total movement of .18
        leftFoundationClaw.setPosition(.18);
    }

    /**
     * Lift the foundation claws to folded position.
     */
    public void retractClaws() {
        // Right starts at 1 and moves to .40 for a total movement of .57
        rightFoundationClaw.setPosition(1);
        // Left starts at 0 and moves to .18 for a total movement of .18
        leftFoundationClaw.setPosition(0);
    }

    /**
     * Raises the servo to release the capstone.
     */
    public void releaseCapstone() {

        capstoneClaw.setPosition(0.1);
    }

    /**
     * Lowers the servo to lock onto the capstone.
     */
    public void grabCapstone() {

        capstoneClaw.setPosition(0.7);
    }



    //------------------------------------------------------------------------------------------------------------------
    // isDone Method:
    //------------------------------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        // There's nothing to do, so isDone() always true.
        return true;
    }
}