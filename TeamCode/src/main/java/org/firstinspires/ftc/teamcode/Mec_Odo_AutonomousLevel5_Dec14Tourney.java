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
    private boolean isRedAlliance = true;
    private boolean isBuildZone = false;
    private boolean isParkAtWall = false;

    private CatHW_Vision.skyStonePos skyStonePos = CatHW_Vision.skyStonePos.RIGHT;

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
            telemetry.addData("Label", robot.eyes.giveSkyStonePos());

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

        //go to block and pick it up

        robot.driveOdo.translateDrive(0,6,CatHW_DriveBase.DRIVE_SPEED,0,.3,1.5);
        robot.driveOdo.waitUntilDone();
        switch (skyStonePos) {
            case LEFT:
                robot.driveOdo.translateDrive(-9, 25, CatHW_DriveOdo.DRIVE_SPEED, -35, .4, 3);
                robot.driveOdo.waitUntilDone();
                robot.jaws.intakeJaws();
                robot.driveOdo.translateDrive(-9, 39, CatHW_DriveOdo.CHILL_SPEED, -35, .25, 1.5);
                robot.driveOdo.waitUntilDone();
                robot.driveOdo.translateDrive(3, 24, CatHW_DriveOdo.DRIVE_SPEED, -35, .25, 1.5);
                break;
            case CENTER:
                robot.driveOdo.translateDrive(0, 25, CatHW_DriveOdo.DRIVE_SPEED, -35, .4, 3);
                robot.driveOdo.waitUntilDone();
                robot.jaws.intakeJaws();
                robot.driveOdo.translateDrive(0, 39, CatHW_DriveOdo.CHILL_SPEED, -35, .25, 1.5);
                robot.driveOdo.waitUntilDone();
                robot.driveOdo.translateDrive(6, 24, CatHW_DriveOdo.DRIVE_SPEED, -35, .25, 1.5);
                break;
            case RIGHT:
                robot.driveOdo.translateDrive(5, 25, CatHW_DriveOdo.DRIVE_SPEED, -35, .4, 3);
                robot.driveOdo.waitUntilDone();
                robot.jaws.intakeJaws();
                robot.driveOdo.translateDrive(5, 39, CatHW_DriveOdo.CHILL_SPEED, -35, .25, 1.5);
                robot.driveOdo.waitUntilDone();
                robot.driveOdo.translateDrive(9, 24, CatHW_DriveOdo.DRIVE_SPEED, -35, .25, 1.5);
                break;
        }

        robot.driveOdo.waitUntilDone();
        // go to build zone

        robot.driveOdo.translateDrive(45,-20,.8,-105,.4,4);
        robot.driveOdo.waitUntilDone();
        robot.jaws.outputJaws();
        robot.driveOdo.translateDrive(45,-13,CatHW_DriveBase.DRIVE_SPEED,-105,.55,2);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(45,-7,.55,-170,.65,5);
        robot.driveOdo.waitUntilDone();


    }
    public void driveBuildZone() throws InterruptedException {
        // Drive towards build site
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CHILL_SPEED, isRedAlliance ? -18 : 15,1.5);
        robot.driveClassic.waitUntilDone();
        // Drive to Foundation
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, -25, 3.0);
        robot.driveClassic.waitUntilDone();
        // Latch on to foundation
        robot.claw.extendClaw();
        robot.robotWait(0.3);
        // Drive back to Building Zone
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, 60, 3.0);
        robot.driveClassic.waitUntilDone();
        // Put some distance between wall and robot
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, -2, 0.5);
        robot.driveClassic.waitUntilDone();
        robot.robotWait(.1);
        robot.claw.retractClaw();
        robot.robotWait(.2);
        //Rotate ourselves back square
        robot.driveClassic.mecTurn(CatHW_DriveBase.CHILL_SPEED,0,1.0);
        robot.driveClassic.waitUntilDone();
        // Slide out to wall to line up
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? -14 : 0, 5.0);
        robot.driveClassic.waitUntilDone();
        // Slide out to towards the line
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? 42 : -25, 5.0);
        robot.driveClassic.waitUntilDone();
        // Drive ahead and line up with the foundation
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, -13, 2);
        robot.driveClassic.waitUntilDone();

        if(!isRedAlliance) {
            robot.driveClassic.mecTurn(CatHW_DriveBase.CHILL_SPEED, 175, 2);
            robot.driveClassic.waitUntilDone();
        }
        // Push the foundation further into the building zone
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? -11 : -17, 2);
        robot.driveClassic.waitUntilDone();
        // Back up and navigate (park on the taped line)
        if (isParkAtWall) {
            robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? 37 : -42, 1);
            robot.driveClassic.waitUntilDone();
            // Put some distance between wall and robot
            robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? -2 : 2, 3.0);
        } else {
            robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? -5 : 5, 1);
        }
        robot.driveClassic.waitUntilDone();
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, 24, 2);
        robot.driveClassic.waitUntilDone();
    }
}