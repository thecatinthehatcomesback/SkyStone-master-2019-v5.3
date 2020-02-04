/*
        CatHW_Async.java

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
 * Motor channel:  Left  setDrivePowers motor:        "left_rear"  & "left_front"
 * Motor channel:  Right setDrivePowers motor:        "right_rear" & "right_front"
 * And so on...
 */
public class CatHW_Async
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
    CatHW_Claw          claw            = null;
    CatHW_DriveClassic  driveClassic    = null;
    CatHW_DriveOdo      driveOdo        = null;
    CatHW_Jaws          jaws            = null;
    CatHW_Tail          tail            = null;
    CatHW_Vision        eyes            = null;

    /* Constructor */
    public CatHW_Async(){

    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap, LinearOpMode theOpMode, boolean isInitOdo)  throws InterruptedException  {

        // Save Reference to Hardware map
        this.hwMap = hwMap;
        opMode = theOpMode;

        // Give Telemetry for each system we begin to init:
        opMode.telemetry.addData("Initialize","Jaws...");
        opMode.telemetry.update();
        jaws = new CatHW_Jaws(this);
        jaws.init();

        opMode.telemetry.addData("Initialize","Tail...");
        opMode.telemetry.update();
        tail = new CatHW_Tail(this);
        tail.init();

        opMode.telemetry.addData("Initialize","Claw...");
        opMode.telemetry.update();
        claw = new CatHW_Claw(this);
        claw.init();

        if (isInitOdo) {
            opMode.telemetry.addData("Initialize","DriveOdo...");
            opMode.telemetry.update();
            driveOdo = new CatHW_DriveOdo(this);
            driveOdo.init();
        }
        opMode.telemetry.addData("Initialize","DriveClassic...");
        opMode.telemetry.update();
        driveClassic = new CatHW_DriveClassic(this);
        driveClassic.init();

        opMode.telemetry.addData("Initialize","Eyes...");
        opMode.telemetry.update();
        eyes = new CatHW_Vision(this);
        eyes.initVision(this.hwMap);

        opMode.telemetry.addData("Initialize","All Done...  BOOM!");
        opMode.telemetry.update();

        // Blinkin LED stuff //
        lights           = this.hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern          = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        underLights      = this.hwMap.get(RevBlinkinLedDriver.class, "under_blinky");
        underPattern     = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);
        underLights.setPattern(underPattern);
    }

    /**
     * ---   ____________________________   ---
     * ---   Common Miscellaneous Methods   ---
     * ---  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/   ---
     */
    public void spawnWait(CatHW_Subsystem subsystem) {
        CatThreadAsync theThread = new CatThreadAsync(subsystem);
        theThread.start();
    }
    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opMode.opModeIsActive()  &&  (delayTimer.seconds() < seconds)) {
            opMode.idle();
        }
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