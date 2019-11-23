/**
 Mec_AutonomousLevel1_Ri2W.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of with gamepad1 and
 detect the gold with DogeCV, hit the right element, place our team marker
 in our depot and park in either crater according to what we say using gamepad1
 at the beginning of the match.

 MecBasic is written to use the most basic approach to our autonomous route      //TODO Change this
 with the help of mechanical sorting jaws and a servo to drop our team marker
 off in the depot.  This autonomous is used for our first qualifier this year
(November 16, 2019).

 This was the old code for our Robot in 2 Weeks bot, but we completely stripped
 it out as with our old bot.



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="Nov 16 Autonomous", group="CatAuto")
public class Mec_AutonomousLevel2_Nov16Tourney extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isBuildZone = true;
    private boolean isParkAtWall = false;

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

        // Do stuff for Loading Zone:
        if (isRedAlliance) {
            robot.driveClassic.mecDriveHorizontal(CatHW_DriveBase.CHILL_SPEED, isParkAtWall ? -1.0 : -28, 3);
            robot.driveClassic.waitUntilDone();
        } else {
            robot.driveClassic.mecDriveHorizontal(CatHW_DriveBase.CHILL_SPEED, isParkAtWall ? 1.0 : 28, 3);
            robot.driveClassic.waitUntilDone();
        }
        // Back up to bridge
        robot.driveClassic.mecDriveVertical(CatHW_DriveBase.CHILL_SPEED, -28.0, 4);
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