/*
        CatAsyncHW.java

    An "hardware" class that acts as the master in which all the other
    "hardwares" run through.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode.
 *
 * This class is used to define all the other hardwares.
 *
 *
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * Note:  All names are lower case and have underscores between words.
 *
 * Motor channel:  Left  drive motor:        "left_rear"  & "left_front"
 * Motor channel:  Right drive motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatAsyncHW
{
    /* Public OpMode members. */

    // LED stuff:
    public RevBlinkinLedDriver lights = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public RevBlinkinLedDriver underLights = null;
    public RevBlinkinLedDriver.BlinkinPattern underPattern;
    public static boolean isRedAlliance = true;


    /* local OpMode members. */
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;


    /* Other Hardware subSystems */
    CatIntakeHW intake  = null;
    CatDriveHW drive    = null;
    CatTailHW tail      = null;


    /* Constructor */
    public CatAsyncHW(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode)  throws InterruptedException  {

        // Save Reference to Hardware map
        hwMap = ahwMap;
        opMode = theOpMode;

        // Give Telemetry for each system we begin to init:
        opMode.telemetry.addData("Initialize","Intake...");
        opMode.telemetry.update();
        intake = new CatIntakeHW(this);
        intake.init();
        opMode.telemetry.addData("Initialize","Drive...");
        opMode.telemetry.update();
        drive = new CatDriveHW(this);
        drive.init();
        opMode.telemetry.addData("Initialize","Tail...");
        opMode.telemetry.update();
        tail = new CatTailHW(this);
        tail.init();opMode.telemetry.addData("Initialize","All Done...  BOOM!");
        opMode.telemetry.update();



        // Blinkin LED stuff //
        lights           = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern          = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        underLights      = hwMap.get(RevBlinkinLedDriver.class, "under_blinky");
        underPattern     = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);
        underLights.setPattern(underPattern);
    }

    /**
     * ---   ____________________________   ---
     * ---   Common Miscellaneous Methods   ---
     * ---  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    public void spawnWait(CatSubsystemHW subsystem) {
        CatAsyncThread theThread = new CatAsyncThread(subsystem);
        theThread.start();
    }
    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opMode.opModeIsActive()  &&  (delayTimer.seconds() < seconds)) {
            opMode.idle();
        }
    }
    public double limitRange(double number, double min, double max) {
        return Math.min(Math.max(number, min), max);
    }

    /**
     * ---   ____________________   ---
     * ---   Color Sensor Methods   ---
     * ---   \/ \/ \/ \/ \/ \/ \/   ---
     */
    public boolean isColorRed(ColorSensor sensorToUse) {
        /**
         * Compare red and blue to decide which is seen
         */

        boolean isRed;

        // Just a simple caparison of the two colors to see which one is seen (probably)
        if (sensorToUse.red() > sensorToUse.blue()) {
            isRed = true;
        } else {
            isRed = false;
        }
        return isRed;
    }
    public int findBaseDelta(ColorSensor colorSensor) {
        /**
         * Before starting to look for a line, find the the current Alpha
         * to add to the threshold so as to give wiggle-room to finding the
         * line.
        */
        int baseDelta = Math.abs(colorSensor.red() - colorSensor.blue());
        return baseDelta;
    }
    public boolean findLine(int baseDelta, ColorSensor colorSensor) {
        /**
         * Tell the robot once a color sensor
         * finds a line.
         */
        boolean lineFound;
        // Take the absolute value of the difference of red and blue
        int currentDelta = Math.abs(colorSensor.red() - colorSensor.blue());

        // Check to see if the line is found
        if (currentDelta > (baseDelta + 70)) {
            lineFound = true;
        } else {
            lineFound = false;
        }
        return lineFound;
    }

    /**
     * ---   __________________   ---
     * ---   End of our methods   ---
     * ---   \/ \/ \/ \/ \/ \/    ---
     */
}// End of class bracket