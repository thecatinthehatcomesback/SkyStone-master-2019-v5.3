/**
 Mec_AutonomousLevel4_Dec14Tourney.java

 A Linear OpMode class to be an autonomous method for both Blue & Red alliance
 sides where we pick which side of the alliance bridge we start off at with
 gamepad1 as well as selecting alliance color and whether we park under the
 alliance bridge closer or further from the game field wall.  Also will detect
 the position and deliver the skystone using machine vision.

 Mec_AutonomousLevel4_Dec14Tourney is written to add machine vision and
 skystone delivery to our autonomous route with the help intake jaws that intake  //TODO: Change this...
 a stone at any orientation for a "touch it-own it" approach.  A servo and two
 motors make up TC-73/Bucky's tail a stack stones as well as our team marker.
 This autonomous is used for our first qualifier of our year (December 14, 2019).

 This is a simple update from the Mec_AutonomousLevel2_Nov16Tourney but wasn't
 implemented in time for competition.  Will soon be obsolete with addition of
 odometry wheels.



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Odometry test", group="CatAuto")
public class OdoPositionTest extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private enum testType {bigSquare, driveAndTwist};
    private testType testToRun = testType.bigSquare;

    private CatHW_Vision.skyStonePos skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize the setDrivePowers system variables.  The init() methods of
         * our hardware class does all the work:
         */
        robot.init(hardwareMap, this, true);
        // Init IMU sensor later when the match starts to avoid drifting in the values
        // Init our Machine Vision
        //eyes.initVision(hardwareMap);

        /**
         * Send telemetry message to signify robot getting ready:
         */
        telemetry.addData("Status: ", "Resetting Encoders...");
        telemetry.update();
        //robot.setDrivePowers.resetDriveEncoders();
        //idle();
        //robot.setDrivePowers.setDriveRunToPosition();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at :%7d  :%7d  :%7d  :%7d",
                robot.driveClassic.leftFrontMotor.getCurrentPosition(),
                robot.driveClassic.rightFrontMotor.getCurrentPosition(),
                robot.driveClassic.leftRearMotor.getCurrentPosition(),
                robot.driveClassic.rightRearMotor.getCurrentPosition());
        telemetry.update();

        /**
         * Init Delay Option Select:
         */
        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
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
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (testToRun == testType.bigSquare) {
                    testToRun = testType.driveAndTwist;
                } else {
                    testToRun = testType.bigSquare;
                }
                delayTimer.reset();
            }


            /**
             * Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);
            telemetry.addData("testToRun", testToRun.toString());
            telemetry.update();

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
        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);

        /* Go! */
        switch (testToRun) {
            case driveAndTwist:
                driveAndTwist();
                break;
            case bigSquare:
                bigSquare();
                break;
        }
        robot.driveOdo.updatesThread.stop();
    }

    public void driveAndTwist() throws InterruptedException {

        //go to block and pick it up
        robot.driveOdo.quickDrive(0, 72, .9, 0, .4, 4);
        //robot.driveOdo.quickDrive(0, 72, .9, 90, .4, 4);
       // robot.driveOdo.quickDrive(0, 0, .9, 90, .4, 4);
        robot.driveOdo.quickDrive(0, 0, .9, 0, .4, 4);
       // robot.driveOdo.quickDrive(0, 72, .9, 90, .3, 4);
        //robot.driveOdo.quickDrive(0, 0, .9, 0, .3, 4);
        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
        telemetry.addData("Y Position", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
        telemetry.addData("Orientation (Degrees)", robot.driveOdo.updatesThread.positionUpdate.returnOrientation());
        telemetry.update();
        robot.robotWait(10.0);
    }
    public void bigSquare() throws InterruptedException {

        //go to block and pick it up
        robot.driveOdo.quickDrive(0, 96, .4, 0, .3, 4);
        robot.driveOdo.quickDrive(48+14, 96, .9, 0, .3, 4);
        robot.driveOdo.quickDrive(48+14, 0, .9, 0, .3, 4);
        robot.driveOdo.quickDrive(0, 0, .4, 0, .3, 4);
        robot.robotWait(1.5);
        //attemt to improve
        //robot.driveOdo.quickDrive(0, 0, .4, 0, 0, 4);
        //robot.driveOdo.quickDrive(0, 0, .4, 0, .9, 4);

        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
        telemetry.addData("Y Position", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
        telemetry.addData("Orientation (Degrees)", robot.driveOdo.updatesThread.positionUpdate.returnOrientation());
        telemetry.update();
        robot.robotWait(10.0);
    }

}