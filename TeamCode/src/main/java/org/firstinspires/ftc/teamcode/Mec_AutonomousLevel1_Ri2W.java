/**
 Mec_AutonomousLevel1_Ri2W.java

 A Linear OpMode class to be an autonomous method for both Blue & Red where
 we pick which side of the lander we are hanging off of with gamepad1 and
 detect the gold with DogeCV, hit the right element, place our team marker
 in our depot and park in either crater according to what we say using gamepad1
 at the beginning of the match.

 MecBasic is written to use the most basic approach to our autonomous route
 with the help of mechanical sorting jaws and a servo to drop our team marker
 off in the depot.  This autonomous is used for our first qualifier this year
(November 10, 2018).

 This was the old code for our Robot in 2 Weeks bot, but we completely stripped
 it OUT as with our old bot.



 This file is a modified version from the FTC SDK.
 Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@Autonomous(name="Ri2W Autonomous", group="CatAuto")
public class Mec_AutonomousLevel1_Ri2W extends LinearOpMode {

    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardwares init here
    //CatHW_Vision eyes  = new CatHW_Vision();   // Vision init
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isBuildZone = true;
    private boolean isParkAtWall = false;

    // Encoder Positions
    static final int MOUTH_CLOSE = -140;
    static final int MOUTH_OPEN = -62;
    static final int MOUTH_LATCH = -125;
    static final int MOUTH_RELEASE = 150;

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

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }

            telemetry.addData("isBuildZone Side?", isBuildZone);
            telemetry.addData("isPartAtWall", isParkAtWall);
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
        if (isBuildZone) {
            driveBuildZone();
        } else {
            driveLoadingZone();
        }
    }
    public void driveLoadingZone() throws InterruptedException {
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Open jaws and start driving forwards towards stone
        robot.jaws.runtime.reset();
        robot.jaws.leftJawMotor.setTargetPosition(MOUTH_OPEN);
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jaws.leftJawMotor.setPower(0.5);
        robot.jaws.waitUntilDone();
        robot.jaws.leftJawMotor.setPower(0);
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, 25, 2);
        robot.driveClassic.waitUntilDone();
        // Drive forward a bit more but slower
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CREEP_SPEED, 10, 3);
        robot.driveClassic.waitUntilDone();

        // Close jaws
        robot.jaws.runtime.reset();
        robot.jaws.leftJawMotor.setTargetPosition(MOUTH_CLOSE);
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jaws.leftJawMotor.setPower(0.5);
        robot.jaws.waitUntilDone();
        robot.jaws.leftJawMotor.setPower(0.1);
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //back up from stones
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, -14, 2);
        robot.driveClassic.waitUntilDone();
        // Drive completely across the taped line
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.DRIVE_SPEED, (isRedAlliance) ? -40 : 40, 6);
        robot.driveClassic.waitUntilDone();

        // Spit block OUT
        robot.jaws.runtime.reset();
        robot.jaws.leftJawMotor.setTargetPosition(MOUTH_RELEASE);
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jaws.leftJawMotor.setPower(0.5);
        robot.jaws.waitUntilDone();
        robot.jaws.leftJawMotor.setPower(0);
        robot.jaws.leftJawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CREEP_SPEED,10, 4);
        robot.driveClassic.waitUntilDone();
        //back up from stone
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CREEP_SPEED, -6, 1);
        robot.driveClassic.waitUntilDone();
        // Navigate (Park over taped line)
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, (isRedAlliance) ? 15 : -15, 3);
        robot.driveClassic.waitUntilDone();



    }
    public void driveBuildZone() throws InterruptedException {
        // Drive to Foundation
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, -30, 3.0);
        robot.driveClassic.waitUntilDone();
        //robot.robotWait(1);
        // Latch on
        robot.claw.extendClaws();
        robot.claw.waitUntilDone();
        robot.robotWait(0.3);
        // Drive back to Building Zone
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, 30, 3.0);
        robot.driveClassic.waitUntilDone();
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, 7, 1.25);
        robot.driveClassic.waitUntilDone();
        robot.robotWait(.1);
        robot.claw.retractClaws();
        robot.robotWait(.2);
        // Slide OUT to towards the line
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, (isRedAlliance) ? -22 : 22, 5.0);
        robot.driveClassic.waitUntilDone();
        // Drive ahead and line up with the foundation
        robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, -18, 2);
        robot.driveClassic.waitUntilDone();
        // Push the foundation further into the building zone
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, (isRedAlliance) ? 8 : -8, 1);
        robot.driveClassic.waitUntilDone();
        // Back up and navigate (park on the taped line)
        if (isParkAtWall) {
            robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, -5, 1);
        } else {
            robot.driveClassic.mecDriveVertical(CatHW_DriveClassic.CHILL_SPEED, 25, 1);
        }
        robot.driveClassic.waitUntilDone();
        robot.driveClassic.mecDriveHorizontal(CatHW_DriveClassic.CHILL_SPEED, (isRedAlliance) ? -20 : 20, 2);
        robot.driveClassic.waitUntilDone();
    }
}
