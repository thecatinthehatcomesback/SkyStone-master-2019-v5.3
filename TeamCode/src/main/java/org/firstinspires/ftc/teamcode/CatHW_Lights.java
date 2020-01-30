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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

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
public class CatHW_Lights implements Runnable
{

    // Thread run condition
    private boolean isRunning   = true;
    private int     sleepTime   = 25;
    private LightPattern defaultPattern = new LightPattern(sleepTime,RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

    // blinkin objects:
    public RevBlinkinLedDriver lights = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public RevBlinkinLedDriver underLights = null;
    public RevBlinkinLedDriver.BlinkinPattern underPattern;


    //hw map
    HardwareMap hwMap;

    //list
    private ArrayList<LightPattern> patternList = new ArrayList<LightPattern>();

    /* Constructor */
    public CatHW_Lights(CatHW_Async mainHardware){

    hwMap = mainHardware.hwMap;

    }


    public void init(){

        // Blinkin LED stuff //
        lights           = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        pattern          = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        underLights      = hwMap.get(RevBlinkinLedDriver.class, "under_blinky");
        underPattern     = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);
        underLights.setPattern(underPattern);

    }

    public void blink (int num, RevBlinkinLedDriver.BlinkinPattern color, int timeperblink){
        for (int i = 0; i < num; i++){
            addQueue(new LightPattern(timeperblink,color));
            addQueue(new LightPattern(timeperblink, RevBlinkinLedDriver.BlinkinPattern.BLACK));

        }

    }

    public void setDefaultColor(RevBlinkinLedDriver.BlinkinPattern color){
        defaultPattern.setPattern(color);
    }



    public void addQueue(LightPattern lp) {
        synchronized (this) {
            //adds the lightpattern to the end of the list
            patternList.add(lp);

        }
    }

    public LightPattern readQueue(){
        synchronized (this) {
         //gets the lightpattern's pattern and delay

        if (patternList.size() > 0) {
            return patternList.remove(0);

        }else {
            return defaultPattern;
        }

        }
    }

    @Override
    public void run() {
        while(isRunning) {

            LightPattern current = readQueue();

            //do stuff
            if (current.getPattern() != pattern){
                lights.setPattern(current.getPattern());
                pattern = current.getPattern();
            }

            //add delay
            try {
                Thread.sleep(current.getDelayMs());
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void stop(){ isRunning = false; }


}// End of class bracket