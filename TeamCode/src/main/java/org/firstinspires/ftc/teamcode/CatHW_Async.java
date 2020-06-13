package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A "hardware" class that acts as the master in which all the other "hardware" classes will run
 * through.  When this class is created, it creates versions of all the other hardware classes used
 * to run the robot.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */
public class CatHW_Async
{
    //----------------------------------------------------------------------------------------------
    // Attributes:
    //----------------------------------------------------------------------------------------------

    /**
     * Attribute that is used to tell the robot through all the other classes whether it is on the
     * red or blue alliance.
     */
    public static boolean isRedAlliance = true;


    /*
    Default / Package-Private OpMode members.  (Since no visibility was inputted, any class in this
    package--AKA the teamcode package--can see these members.)
     */
    HardwareMap hwMap = null;
    LinearOpMode opMode = null;


    /*
    The hardware subsystems that this class "owns" a copy of:
     */
    CatHW_Jaws jaws = null;
    CatHW_Claw claw = null;
    CatHW_DriveOdo drive = null;
    CatHW_Tail tail = null;
    CatHW_Vision eyes = null;
    CatHW_Lights lights = null;



    //----------------------------------------------------------------------------------------------
    // Setup Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Initialize all the standard Hardware interfaces as well as all the subsystem hardware
     * classes.
     *
     * @param ahwMap is for saving a reference to / remembering the robot's hardware map.
     * @param theOpMode is for saving a reference to / remembering the Linear OpMode usage.
     * @throws InterruptedException in case of any errors.
     */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode)  throws InterruptedException {

        // Save a reference to hardware map and opMode:
        hwMap = ahwMap;
        opMode = theOpMode;

        /*
        Give Telemetry for each system we begin to init:
         */
        opMode.telemetry.addData("Initialize", "Jaws...");
        opMode.telemetry.update();
        jaws = new CatHW_Jaws(this);
        jaws.init();

        opMode.telemetry.addData("Initialize", "Tail...");
        opMode.telemetry.update();
        tail = new CatHW_Tail(this);
        tail.init();

        opMode.telemetry.addData("Initialize", "Claw...");
        opMode.telemetry.update();
        claw = new CatHW_Claw(this);
        claw.init();

        opMode.telemetry.addData("Initialize", "Lights...");
        opMode.telemetry.update();
        lights = CatHW_Lights.getInstanceAndInit(this);
        lights.init();

        opMode.telemetry.addData("Initialize", "Drive...");
        opMode.telemetry.update();
        drive = new CatHW_DriveOdo(this);
        drive.init();

        // TODO:  Uncomment this out at some point so that the machine vision works again.
        /*opMode.telemetry.addData("Initialize", "Eyes...");
        opMode.telemetry.update();
        eyes = new CatHW_Vision(this);
        eyes.initVision(hwMap);*/

        // All hardware classes are initialized!
        opMode.telemetry.addData("Initialize", "All Done...  BOOM!");
        opMode.telemetry.update();
    }



    //----------------------------------------------------------------------------------------------
    // Common Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Method which will pause the robot's action for a set amount of seconds.  Used in between
     * actions that could either take time or when some part of the robot just needs to be put on
     * hold.
     *
     * @param seconds that the robot's systems will be delayed.
     */
    public void robotWait(double seconds) {

        ElapsedTime delayTimer = new ElapsedTime();

        while (opMode.opModeIsActive() && (delayTimer.seconds() < seconds)) {
            opMode.idle();
        }
    }
}