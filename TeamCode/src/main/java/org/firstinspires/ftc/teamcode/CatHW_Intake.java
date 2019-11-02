/*
        CatHW_Intake.java

    A "hardware" class containing common code accessing hardware specific
    to the movement and rotation of the intake.  This is a modified and
    stripped down version of CatSingleOverallHW to run all of intake
    movements.  This file is used by the new autonomous OpModes to run
    multiple operations at once.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the intake specific hardware for the robot to
 * allow for multiple operations during autonomous.  In this case, that robot is //todo Change this name
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
public class CatHW_Intake extends CatHW_Subsystem
{

    /* Public OpMode members. */
    static final int MOUTH_CLOSE = -140;
    static final int MOUTH_OPEN = -62;
    static final int MOUTH_LATCH = -125;
    static final int MOUTH_RELEASE = 150;


    // Motors:
    public DcMotor intakeMotor = null;


    // Timers:
    ElapsedTime runtime = new ElapsedTime();

    /* local OpMode members. */



    /* Constructor */
    public CatHW_Intake(CatHW_Async mainHardware){
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init()  throws InterruptedException  {

        // Define and Initialize Motors //
        intakeMotor = hwMap.dcMotor.get("intake_motor");

        // Set Motor and Servo Modes //
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeMove(int target) {
        intakeMotor.setTargetPosition(target);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        intakeMotor.setPower(0.5);
        isDone();
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void pacManStoneEater() {
        /**
         * During autonomous, we use this to turn on the
         * intake and eat the stones!
         */
    }


    /* isDone stuff for intakeHW */
    static  double TIMEOUT = 3.0;
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("intake power %.2f,",intakeMotor.getPower()));
        Log.i("catbot", String.format("intake encoder " + intakeMotor.getCurrentPosition()));
        return !(intakeMotor.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}// End of class bracket