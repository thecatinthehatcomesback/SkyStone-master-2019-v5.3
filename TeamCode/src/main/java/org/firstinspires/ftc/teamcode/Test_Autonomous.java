/*
        Test_Autonomous.java

    A Linear OpMode class to be place to test code both old and new.  We constantly edit this,
    taking out and adding in code.  This is never the same at any given time.


    This file has been modified from the original FTC SkyStone SDK.
    Written by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test_Autonomous", group="CatTest Auto")
public class Test_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;

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
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }

            /**
             * LED code:
             */
            if (isRedAlliance) {

            } else {

            }

            /**
             * Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
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

        /* Go! */

        robot.driveOdo.translateDrive(0,36, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        /*robot.driveOdo.translateDrive(0,72, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,72, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();*/


        // Testing the new mecTurn
        //robot.driveClassic.mecTurn(CatHW_DriveClassic.TURN_SPEED, 170, 3.0);
    }
}
