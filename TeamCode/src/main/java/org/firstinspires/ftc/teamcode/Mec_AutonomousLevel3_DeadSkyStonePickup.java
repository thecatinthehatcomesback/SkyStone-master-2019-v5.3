/**
 Mec_AutonomousLevel3_DeadSkyStonePickup.java

 A Linear OpMode class to be an autonomous method for both Blue & Red alliance
 sides where we pick which side of the alliance bridge we start off at with
 gamepad1 as well as selecting alliance color and whether we park under the
 alliance bridge closer or further from the game field wall.  Also will detect
 the position and deliver the skystone using machine vision.

 Mec_AutonomousLevel3_DeadSkyStonePickup is written to add machine vision and
 skystone delivery to our autonomous route with the help intake jaws that intake
 a stone at any orientation for a "touch it-own it" approach.  A servo and two
 motors make up TC-73/Bucky's tail a stack stones as well as our team marker.
 This autonomous is used for our first qualifier of our year (November 16, 2019).

 This is a simple update from the Mec_AutonomousLevel2_Nov16Tourney but wasn't
 implemented in time for competition.  Will soon be obsolete with addition of
 odometry wheels.



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="Dead SkyStone Pickup Autonomous", group="CatAuto")
public class Mec_AutonomousLevel3_DeadSkyStonePickup extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isBuildZone = true;
    private boolean isParkAtWall = false;

    private CatHW_Vision.skyStonePos skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize the setDrivePowers system variables.  The init() methods of
         * our hardware class does all the work:
         */
        robot.init(hardwareMap, this, false);
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

        // Drive to quarry
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CHILL_SPEED,22,2);
        robot.driveClassic.waitUntilDone();
        // Set the tongue to stop the block from being forced through so we can spit back OUT later
        //robot.jaws.pusherMid();
        // Turn and grab the SkyStone
        switch (skyStonePos) {
            case INSIDE:
                robot.driveClassic.mecDriveHorizontal(CatHW_DriveBase.CHILL_SPEED, -3, 1);
                robot.driveClassic.waitUntilDone();
                robot.driveClassic.mecTurn(CatHW_DriveBase.CHILL_SPEED, -35, 1.5);
                robot.jaws.intakeJaws();
                robot.driveClassic.waitUntilDone();
                break;
            case CENTER:
                robot.driveClassic.mecDriveHorizontal(CatHW_DriveBase.CHILL_SPEED, -7, 1);
                robot.driveClassic.waitUntilDone();
                robot.driveClassic.mecTurn(CatHW_DriveBase.CHILL_SPEED, -35, 1.5);
                robot.jaws.intakeJaws();
                robot.driveClassic.waitUntilDone();
                break;
            case OUTSIDE:
                robot.driveClassic.mecDriveHorizontal(CatHW_DriveBase.CHILL_SPEED, -15, 1);
                robot.driveClassic.waitUntilDone();
                robot.driveClassic.mecTurn(CatHW_DriveBase.CHILL_SPEED, -35, 1.5);
                robot.jaws.intakeJaws();
                robot.driveClassic.waitUntilDone();
                break;
        }
        // Intake stone (hopefully a SkyStone)
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CREEP_SPEED,10,3);
        robot.driveClassic.waitUntilDone();
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CREEP_SPEED,-5,2);
        robot.driveClassic.waitUntilDone();
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.DRIVE_SPEED,6,1.5);
        robot.driveClassic.waitUntilDone();
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CHILL_SPEED,-12,2);
        robot.driveClassic.waitUntilDone();
        //robot.jaws.turnOffJaws();
        robot.driveClassic.mecTurn(CatHW_DriveBase.TURN_SPEED, 90, 2.5);
        robot.driveClassic.waitUntilDone();
        // Drive into building zone
        switch (skyStonePos ){

            case INSIDE:
                robot.driveClassic.mecDriveVertical(CatHW_DriveBase.DRIVE_SPEED, 26, 2);
                robot.driveClassic.waitUntilDone();
                break;
            case CENTER:
                robot.driveClassic.mecDriveVertical(CatHW_DriveBase.DRIVE_SPEED, 22, 2);
                robot.driveClassic.waitUntilDone();
                break;
            case OUTSIDE:
                robot.driveClassic.mecDriveVertical(CatHW_DriveBase.DRIVE_SPEED, 18, 2);
                robot.driveClassic.waitUntilDone();
                break;

        }
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.DRIVE_SPEED, 10, 2);
        robot.driveClassic.waitUntilDone();
        robot.jaws.outputJaws();
        robot.robotWait(2.0);
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.DRIVE_SPEED, -10, 2);
        robot.driveClassic.waitUntilDone();

    }
    public void driveBuildZone() throws InterruptedException {
        // Drive towards build site
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CHILL_SPEED, isRedAlliance ? -18 : 15,1.5);
        robot.driveClassic.waitUntilDone();
        // Drive to Foundation
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, -25, 3.0);
        robot.driveClassic.waitUntilDone();
        // Latch on to foundation
        robot.claw.extendClaws();
        robot.robotWait(0.3);
        // Drive back to Building Zone
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, 60, 3.0);
        robot.driveClassic.waitUntilDone();
        // Put some distance between wall and robot
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, -2, 0.5);
        robot.driveClassic.waitUntilDone();
        robot.robotWait(.1);
        robot.claw.retractClaws();
        robot.robotWait(.2);
        //Rotate ourselves back square
        robot.driveClassic.mecTurn(CatHW_DriveBase.CHILL_SPEED,0,1.0);
        robot.driveClassic.waitUntilDone();
        // Slide OUT to wall to line up
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, isRedAlliance ? -14 : 0, 5.0);
        robot.driveClassic.waitUntilDone();
        // Slide OUT to towards the line
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