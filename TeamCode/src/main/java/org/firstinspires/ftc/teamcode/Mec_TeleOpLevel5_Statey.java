/*
    Mec_TeleOpLevel5_Statey.java

    A Linear opmode class that is used as our
    TeleOp method for the driver controlled period.

    By FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "State TeleOp", group = "CatTeleOp")
public class Mec_TeleOpLevel5_Statey extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    private ElapsedTime stoneReleaseTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatHW_Async robot = null;  // Use our new mecanum async hardware


    // Our constructor for this class
    public Mec_TeleOpLevel5_Statey() {
        robot = new CatHW_Async();

    }

    @Override
    public void runOpMode() throws InterruptedException {


        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this, false);
        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (robot.isRedAlliance) {
            robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        } else {
            robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
        }

        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        stoneReleaseTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean alreadyStone = true;
        boolean endGame = false;
        boolean under10Sec = false;

        robot.tail.openGrabber();

        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Drive train speed control:
            if (gamepad1.left_bumper) {
                driveSpeed = 1.00;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.30;
            } else {
                driveSpeed = 0.70;
            }

            // Input for setDrivePowers train and sets the dead-zones:
            leftFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;
            leftBack = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightBack = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;

            // Calculate the scale factor:
            SF = robot.driveClassic.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront = leftFront * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack = leftBack * SF * driveSpeed;
            rightBack = rightBack * SF * driveSpeed;
            // DRIVE!!!
            robot.driveClassic.setDrivePowers(leftFront, rightFront, leftBack, rightBack);

            // Jaws Control:
            if (gamepad1.left_bumper) {
                robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger));
            } else {
                robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.3));
            }

            // Open/Close Foundation Fingers:
            foundationHooks(gamepad1);

            //capstone servo
            capstone(gamepad1);


            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */


            // Tail/Stacker lift motor controls:
            robot.tail.tailLift.setPower(gamepad2.right_stick_y);
            robot.tail.tailLift2.setPower(gamepad2.right_stick_y);

            // Extend controls:
            robot.tail.tailExtend.setPower(-gamepad2.left_stick_y);

            // Grabber controls:
            if (gamepad2.left_bumper) {
                robot.tail.closeGrabber();
            }
            if (gamepad2.right_bumper) {
                robot.tail.openGrabber();
            }


            //  if driver 1 isn't using jaws, let driver 2 set Jaws Control:
            if (gamepad1.right_trigger - (gamepad1.left_trigger) == 0) {
                robot.jaws.setJawPower(gamepad2.right_trigger - (gamepad2.left_trigger * 0.3));
            }

            // Open/Close Foundation Fingers:
            foundationHooks(gamepad2);

            //capstone servo
            capstone(gamepad2);

            //add lights

            //code to add green blink if it picks up a stone
            if (robot.jaws.hasStone() && !alreadyStone) {
                robot.lights.blink(4, RevBlinkinLedDriver.BlinkinPattern.YELLOW, 150);
                alreadyStone = true;
            }
            if (alreadyStone && !robot.jaws.hasStone()) {
                alreadyStone = false;
            }

            //code to blink at endgame
            if (!endGame && elapsedGameTime.seconds() > 90) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                endGame = true;
            }
            if (!under10Sec && elapsedGameTime.seconds() > 110) {
                under10Sec = true;
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 1200);
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 1000);
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 800);
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 600);
                robot.lights.blink(2, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 400);
                robot.lights.blink(3, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 200);
                robot.lights.blink(10, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, 125);
            }


            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);
            //telemetry.addData("Intake Power:","%.2f", robot.jaws.leftJawMotor.getPower());

            telemetry.addData("Encoder lf/lr rf/rr", "%5d %5d   %5d %5d",
                    robot.driveClassic.leftFrontMotor.getCurrentPosition(),
                    robot.driveClassic.leftRearMotor.getCurrentPosition(),
                    robot.driveClassic.rightFrontMotor.getCurrentPosition(),
                    robot.driveClassic.rightRearMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.driveOdo.updatesThread.stop();
    }

    //foundation hooks and capstone code for both controllers
    public void foundationHooks(Gamepad controller){
        // Open/Close Foundation Fingers:
        if (controller.b) {
            robot.claw.retractClaws();
        }

        if (controller.a) {
            robot.claw.extendClaws();
        }
    }
    public void capstone(Gamepad controller){

        //capstone servo
        if (controller.x) {
            robot.claw.releaseCapstone();
        }
        if (controller.y) {
            robot.claw.grabCapstone();
        }

    }
}
