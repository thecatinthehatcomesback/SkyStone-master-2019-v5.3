package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Test_Autonomous.java
 *
 *
 * A Linear OpMode class to be place to test code both old and new.  We constantly edit this, taking
 * out and adding in code.  This is never the same at any given time.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@Autonomous(name="Test_Autonomous", group="CatTest Auto")
public class Test_Autonomous extends LinearOpMode
{
    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes initialize here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        robot.init(hardwareMap, this);


        //------------------------------------------------------------------------------------------
        // Init Delay Option Select:
        //------------------------------------------------------------------------------------------

        // After init is pushed but before Start we can change the delay using dpad up/down: //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() ) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isRedAlliance) {
                    isRedAlliance = false;
                    CatHW_Async.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    CatHW_Async.isRedAlliance = true;
                }
                delayTimer.reset();
            }


            /*
            Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
            telemetry.update();

            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */
        }



        //------------------------------------------------------------------------------------------
        // Run After Play Is Pressed:
        //------------------------------------------------------------------------------------------

        /*
        Init the IMU after play so that it is not offset after remaining idle for a minute or two...
         */
        robot.drive.IMU_Init();

        /* Go! */

        ArrayList<CatType_CurvePoint> allPoints = new ArrayList<>();

        /*allPoints.add(new CurvePoint(0, 0, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(0, 96, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(72, 96, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(72, 0, 3.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(0, 0, 3.0, 1.0, Math.toRadians(0), 1.0));*/

        /*robot.drive.pursuitDrive(allPoints, .7, 3.0, 8);
        robot.drive.waitUntilDone();*/
    }
}
