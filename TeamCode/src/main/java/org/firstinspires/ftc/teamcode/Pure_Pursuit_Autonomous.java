package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isBuildZone = false;
    private boolean isParkAtWall = false;
    private boolean isFoundation = true;

    private CatHW_Vision.skyStonePos skyStonePos = CatHW_Vision.skyStonePos.OUTSIDE;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        robot.init(hardwareMap, this, true);


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
            if (((gamepad1.dpad_right) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isFoundation) {
                    isFoundation = false;
                } else {
                    isFoundation = true;
                }
                delayTimer.reset();
            }

            /**
             * LED code:
             */
            if (robot.isRedAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            } else {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
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

            telemetry.addData("isBuildZone Side? ", isBuildZone);
            telemetry.addData("isParkAtWall ", isParkAtWall);
            telemetry.addData("isFoundation ", isFoundation);
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


        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        allPoints.add(new CurvePoint(0, 0, 10.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(0, 96, 10.0, 1.0, Math.toRadians(0), 1.0));
        allPoints.add(new CurvePoint(35, 96, 10.0, 1.0, Math.toRadians(0), 1.0));
        //allPoints.add(new CurvePoint(72, 0, 3.0, 1.0, Math.toRadians(0), 1.0));
        //allPoints.add(new CurvePoint(0, 0, 3.0, 1.0, Math.toRadians(0), 1.0));

        robot.driveOdo.translateDrive(allPoints, .7, 0, 3.0, 8);
        robot.driveOdo.waitUntilDone();

        robot.driveOdo.updatesThread.stop();
    }
}