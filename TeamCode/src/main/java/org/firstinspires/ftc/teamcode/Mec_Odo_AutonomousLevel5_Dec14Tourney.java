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


@Autonomous(name="Dec 14 Odo Autonomous", group="CatAuto")
public class Mec_Odo_AutonomousLevel5_Dec14Tourney extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = false;
    private boolean isBuildZone = false;
    private boolean isParkAtWall = false;

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

            robot.eyes.findSkyStone();

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
                if (isRedAlliance) {
                    isRedAlliance = false;
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.y) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isBuildZone) {
                    isBuildZone = false;
                } else {
                    isBuildZone = true;
                }
                delayTimer.reset();
            }
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isParkAtWall) {
                    isParkAtWall = false;
                } else {
                    isParkAtWall = true;
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

            skyStonePos = robot.eyes.giveSkyStonePos();
            if (skyStonePos == CatHW_Vision.skyStonePos.OUTSIDE && !isRedAlliance){
                skyStonePos = CatHW_Vision.skyStonePos.CENTER;
            }else if (skyStonePos == CatHW_Vision.skyStonePos.CENTER && !isRedAlliance){
                skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;
            }


            telemetry.addData("Label", skyStonePos);

            telemetry.addData("left position", robot.eyes.lastLeft);
            telemetry.addData("right position", robot.eyes.lastRight);
            telemetry.addData("center position", (robot.eyes.lastRight+robot.eyes.lastLeft)/2);
            telemetry.addData("confidence", robot.eyes.lastConfidence);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            telemetry.addData("isBuildZone Side?", isBuildZone);
            telemetry.addData("isParkAtWall", isParkAtWall);
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
        if (isBuildZone) {
            driveBuildZone();
        } else {
            driveLoadingZone();
        }
    }
    public void driveLoadingZone() throws InterruptedException {

        if(isRedAlliance) {
            //go to block and pick it up
            robot.tail.openGrabber();
            robot.driveOdo.translateDrive(0, 6, .9, 0, .2, 1);
            robot.driveOdo.waitUntilDone();
            robot.jaws.intakeJaws();
            switch (skyStonePos) {
                case INSIDE:
                    robot.driveOdo.translateDrive(isRedAlliance ? 3 : 0, 28, .65, isRedAlliance ? -38 : 38, .45, 2.5);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.5);
                    robot.driveOdo.translateDrive(isRedAlliance ? -7 : 4, 46, .7, isRedAlliance ? -55 : 55, .5, 2);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive(isRedAlliance ? 7 : -4, 24, .8, isRedAlliance ? 70 : -70, .75, 3);
                    robot.driveOdo.waitUntilDone();
                    break;
                case CENTER:
                    robot.driveOdo.translateDrive(isRedAlliance ? 11 : -11, 28, .8, isRedAlliance ? -38 : 38, .45, 2.5);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.55);
                    robot.driveOdo.translateDrive(isRedAlliance ? 4 : -4, 44, .8, isRedAlliance ? -55 : 55, .6, 2);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive(isRedAlliance ? 17 : -17, 24, .8, isRedAlliance ? 70 : -70, .75, 3);
                    robot.driveOdo.waitUntilDone();
                    break;
                case OUTSIDE:
                    robot.driveOdo.translateDrive(isRedAlliance ? 15 : -15, 28, .8, isRedAlliance ? -20 : 20, .45, 2.5);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.35);
                    //collect sky stone
                    robot.driveOdo.translateDrive(isRedAlliance ? 11 : -11, 44, .56, isRedAlliance ? -50 : 50, .65, 2.5);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.translateDrive(isRedAlliance ? 13 : -13, 24, .8, isRedAlliance ? 60 : -60, .7, 3);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();

                    break;
            }

            //drive under Sky bridge
            robot.jaws.turnOffJaws();
            robot.driveOdo.translateDrive(isRedAlliance ? 41 : -41, 26, .8, isRedAlliance ? 90 : -90, .5, 2.5);
            robot.driveOdo.waitUntilDone();
            //release sky stone while backing up
            robot.jaws.outputJaws();
            //robot.driveOdo.translateDrive(25.5,26,.8,90,.2,2);
            //robot.driveOdo.waitUntilDone();

            switch (skyStonePos) {
                case INSIDE:
                    //go to center position
                    robot.driveOdo.translateDrive(isRedAlliance ? -29 : 29, 20, .8, isRedAlliance ? -40 : 40, .55, 4);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.55);
                    //collect sky stone
                    robot.jaws.intakeJaws();
                    robot.driveOdo.translateDrive(isRedAlliance ? -32 : 32, 47, .8, isRedAlliance ? -70 : 70, .66, 2);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive(isRedAlliance ? -17 : 17, 18, .8, isRedAlliance ? 70 : -70, .75, 3);
                    robot.driveOdo.waitUntilDone();
                    break;
                case CENTER:
                    //go to center position
                    robot.driveOdo.translateDrive(isRedAlliance ? -21 : 21, 28, .8, isRedAlliance ? -37 : 37, .45, 4);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.5);
                    //collect sky stone
                    robot.jaws.intakeJaws();
                    robot.driveOdo.translateDrive(isRedAlliance ? -25.5 : 25.5, 49, .7, isRedAlliance ? -52 : 52, .6, 2);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive(isRedAlliance ? -7 : 7, 22, .8, isRedAlliance ? 70 : -70, .75, 4);
                    robot.driveOdo.waitUntilDone();
                    break;
                case OUTSIDE:
                    //go to right position
                    robot.driveOdo.translateDrive(isRedAlliance ? -16 : 16, 28, .8, isRedAlliance ? -20 : 20, .45, 3);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.35);
                    //collect sky stone
                    robot.jaws.intakeJaws();
                    robot.driveOdo.translateDrive(isRedAlliance ? -18 : 18, 47, .56, isRedAlliance ? -55 : 55, .65, 3);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.translateDrive(isRedAlliance ? -11 : 11, 22, .8, isRedAlliance ? 60 : -60, .7, 4);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    break;

            }
            if (skyStonePos == CatHW_Vision.skyStonePos.INSIDE) {
                //drive under Sky bridge and deliver sky stone
                robot.jaws.turnOffJaws();
                robot.driveOdo.translateDrive(isRedAlliance ? 41 : -41, 16.5, .80, isRedAlliance ? 96 : -96, .5, 2.5);
                robot.driveOdo.waitUntilDone();
                //release sky stone while backing up
                robot.jaws.outputJaws();
                //park under sky bridge
                robot.driveOdo.translateDrive(isRedAlliance ? 25.5 : -25.5, 16.5, .85, isRedAlliance ? 96 : -96, .2, 2);
                robot.driveOdo.waitUntilDone();

            } else {
                //drive under Sky bridge and deliver sky stone
                robot.jaws.turnOffJaws();
                robot.driveOdo.translateDrive(isRedAlliance ? 41 : -41, 17, .85, isRedAlliance ? 96 : -96, .5, 2.5);
                robot.driveOdo.waitUntilDone();
                //release sky stone while backing up
                robot.jaws.outputJaws();
                //park under sky bridge
                robot.driveOdo.translateDrive(isRedAlliance ? 25.5 : -25.5, 17, .85, isRedAlliance ? 96 : -96, .2, 2);
                robot.driveOdo.waitUntilDone();

            }



        }else {


            //if is blue



            //go to block and pick it up
            robot.tail.openGrabber();
            robot.driveOdo.translateDrive(0, 6, .9, 0, .2, 1);
            robot.driveOdo.waitUntilDone();
            robot.jaws.intakeJaws();
            switch (skyStonePos) {
                case INSIDE:
                    robot.driveOdo.translateDrive( -4, 26, .65,  38, .45, 2.5);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.5);
                    robot.driveOdo.translateDrive( 0, 44, .7, 55, .5, 2);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive( -8, 22, .8, -70, .75, 3);
                    robot.driveOdo.waitUntilDone();
                    break;
                case CENTER:
                    robot.driveOdo.translateDrive( -11, 26, .8, 38, .45, 2.5);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.55);
                    robot.driveOdo.translateDrive( -4, 42, .8, 55, .6, 2);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive( -17, 22, .8, -70, .75, 3);
                    robot.driveOdo.waitUntilDone();
                    break;
                case OUTSIDE:
                    robot.driveOdo.translateDrive( -17, 26, .8,  20, .45, 2.5);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.35);
                    //collect sky stone
                    robot.driveOdo.translateDrive( -17, 42, .56,  50, .65, 2.5);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.translateDrive( -15, 22, .8, -60, .7, 3);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();

                    break;
            }

            //drive under Sky bridge
            robot.jaws.turnOffJaws();
            robot.driveOdo.translateDrive( -41, 24, .8, -90, .5, 2.5);
            robot.driveOdo.waitUntilDone();
            //release sky stone while backing up
            robot.jaws.outputJaws();

            switch (skyStonePos) {
                case INSIDE:
                    //go to center position
                    robot.driveOdo.translateDrive( 29, 18, .8, 40, .55, 4);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.55);
                    //collect sky stone
                    robot.jaws.intakeJaws();
                    robot.driveOdo.translateDrive( 32, 45, .8, 70, .66, 2);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive( 17, 16, .8, -70, .75, 3);
                    robot.driveOdo.waitUntilDone();
                    break;
                case CENTER:
                    //go to center position
                    robot.driveOdo.translateDrive( 21, 26, .8, 37, .45, 4);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.5);
                    //collect sky stone
                    robot.jaws.intakeJaws();
                    robot.driveOdo.translateDrive( 25.5, 47, .7, 52, .6, 2);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    robot.driveOdo.translateDrive( 7, 20, .8, -70, .75, 4);
                    robot.driveOdo.waitUntilDone();
                    break;
                case OUTSIDE:
                    //go to right position
                    robot.driveOdo.translateDrive( 16, 26, .8, 20, .45, 3);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerBoast(.35);
                    //collect sky stone
                    robot.jaws.intakeJaws();
                    robot.driveOdo.translateDrive( 18, 45, .56, 55, .65, 3);
                    robot.driveOdo.waitUntilDone();
                    //back up from stones
                    robot.driveOdo.translateDrive( 11, 20, .8, -60, .7, 4);
                    robot.driveOdo.waitUntilDone();
                    robot.driveOdo.updatesThread.powerUpdate.powerNormal();
                    break;

            }
            if (skyStonePos == CatHW_Vision.skyStonePos.INSIDE) {
                //drive under Sky bridge and deliver sky stone
                robot.jaws.turnOffJaws();
                robot.driveOdo.translateDrive( -41, 14.5, .80, -96, .5, 2.5);
                robot.driveOdo.waitUntilDone();
                //release sky stone while backing up
                robot.jaws.outputJaws();
                //park under sky bridge
                robot.driveOdo.translateDrive( -25.5, 14.5, .85, -96, .2, 2);
                robot.driveOdo.waitUntilDone();

            } else {
                //drive under Sky bridge and deliver sky stone
                robot.jaws.turnOffJaws();
                robot.driveOdo.translateDrive( -41, 15, .85, -96, .5, 2.5);
                robot.driveOdo.waitUntilDone();
                //release sky stone while backing up
                robot.jaws.outputJaws();
                //park under sky bridge
                robot.driveOdo.translateDrive( -25.5, 15, .85, -96, .2, 2);
                robot.driveOdo.waitUntilDone();

            }
        }


    }
    public void driveBuildZone() throws InterruptedException {

        // drive to the foundation slowly
        robot.driveOdo.translateDrive( isRedAlliance ? -20 : 17,-35,.45,isRedAlliance ? 8 : -8,.2,4);
        robot.driveOdo.waitUntilDone();
        //lower the foundation claws
        robot.claw.extendClaws();
        robot.robotWait(.25);
        //override the min power so we have enough power to move the foundation while driving
        robot.driveOdo.updatesThread.powerUpdate.powerBoast(.7);
        //drive straight forward a little to simplify the turn
        robot.driveOdo.translateDrive(isRedAlliance ? -13 : 13,-19,.9,0,.2,3);
        robot.driveOdo.waitUntilDone();
        //rotate the foundation while moving forward
        robot.driveOdo.translateDrive(isRedAlliance ? -5 : 5,-6,.9,isRedAlliance ? 90 : -90,.7,4);
        robot.driveOdo.waitUntilDone();
        //push the foundation against the wall
        robot.driveOdo.translateDrive(isRedAlliance ? -13 : 13,-6,.9,isRedAlliance ? 90 : -90,.67,3);
        robot.driveOdo.waitUntilDone();
        //reset min power to normal
        robot.driveOdo.updatesThread.powerUpdate.powerNormal();
        //lift up the claw
        robot.claw.retractClaws();
        robot.robotWait(.25);
        if(isParkAtWall){
            robot.driveOdo.translateDrive(isRedAlliance ? -5 : 5, -3, .8, isRedAlliance ? 90 : -90, .2, 2);
            robot.driveOdo.waitUntilDone();
            robot.driveOdo.translateDrive(isRedAlliance ? 25 : -26, isRedAlliance ? -3 : 0, .8, isRedAlliance ? 90 : -90, .2, 2);
            robot.driveOdo.waitUntilDone();
        }
        else {
            robot.driveOdo.translateDrive(isRedAlliance ? -5 : 5, -30, .8, isRedAlliance ? 90 : -90, .2, 2);
            robot.driveOdo.waitUntilDone();
            robot.driveOdo.translateDrive(isRedAlliance ? 22 : -26, isRedAlliance ? -33 : -28, .8, isRedAlliance ? 90 : -90, .2, 2);
            robot.driveOdo.waitUntilDone();
        }

    }
}