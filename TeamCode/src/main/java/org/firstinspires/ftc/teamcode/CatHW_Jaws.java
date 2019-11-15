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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the jaws specific hardware for the robot to
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
public class CatHW_Jaws extends CatHW_Subsystem
{

    /* Public OpMode members. */
    static final double JAW_POWER           = 0.8;
    static final double PUSHER_OPEN         = -1.0;
    static final double PUSHER_MID          = 0.2;
    static final double PUSHER_FULL_PUSH    = 1.0;


    // Motors:
    public DcMotor leftJawMotor     = null;
    public DcMotor rightJawMotor    = null;

    public Servo stonePusher        = null;

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
        leftJawMotor    = hwMap.dcMotor.get("left_jaw_motor");
        rightJawMotor   = hwMap.dcMotor.get("right_jaw_motor");
        stonePusher     = hwMap.servo.get("stone_pusher");

        leftJawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightJawMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Motor and Servo Modes //
        leftJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setJawPower(double power) {
        leftJawMotor.setPower(power);
        rightJawMotor.setPower(power);
    }
    public void intakeJaws() {
        /**
         * Turn on both jaws motors to suck in:
         */

        leftJawMotor.setPower(JAW_POWER);
        rightJawMotor.setPower(JAW_POWER);
    }
    public void outputJaws() {
        /**
         * Turn on both jaws motors to spit out:
         */

        leftJawMotor.setPower(-JAW_POWER);
        rightJawMotor.setPower(-JAW_POWER);
    }
    public void turnOffJaws() {
        /**
         * Turn off both jaws motors:
         */

        leftJawMotor.setPower(0.0);
        rightJawMotor.setPower(0.0);
    }
    public  void pusherRetract(){
        stonePusher.setPosition(PUSHER_OPEN);
    }
    public  void pusherMid(){
        stonePusher.setPosition(PUSHER_MID);
    }
    public  void pusherFullPush(){
        stonePusher.setPosition(PUSHER_FULL_PUSH);
    }


    /* isDone stuff for CatHW_Jaws */
    static double TIMEOUT = 3.0;
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("left jaw power %.2f,", leftJawMotor.getPower()));
        Log.d("catbot", String.format("right jaw power %.2f,", rightJawMotor.getPower()));
        return !(leftJawMotor.isBusy() && rightJawMotor.isBusy() && (runtime.seconds() < TIMEOUT));
    }
}// End of class bracket