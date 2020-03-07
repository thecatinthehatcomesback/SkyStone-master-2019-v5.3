package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */
@Autonomous(name="Pure Pursuit Autonomous", group="CatAuto")
public class Pure_Pursuit_Autonomous extends LinearOpMode
{
    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        robot.init(hardwareMap, this);


        /*
        Init Delay Option Select:
         */
        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }


            /**
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */
        }
        /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */

        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */
        //robot.drive.IMU_Init();

        /* Go! */


        ArrayList<CurvePoint> simpleDrivePath = new ArrayList<>();

        simpleDrivePath.add(new CurvePoint(0, 0,0, 20.0));
        simpleDrivePath.add(new CurvePoint(0, 96, 0, 20.0));
        simpleDrivePath.add(new CurvePoint(60, 96, 0, 20.0));
        simpleDrivePath.add(new CurvePoint(60, 0, 0, 20.0));
        //simpleDrivePath.add(new CurvePoint(0, 0, 0, 20.0));

        robot.drive.translateDrive(simpleDrivePath, .7, 0, 20.0, 16);
        robot.drive.waitUntilDone();

        robot.drive.updatesThread.stop();
    }
}