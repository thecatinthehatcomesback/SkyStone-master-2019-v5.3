/*
        Mec_TeleOpLevel1_Ri2W.java

    A Linear opmode class that is used as our
    TeleOp method for the driver controlled period.

    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp", group="CatTeleOp")
public class Mec_TeleOpLevel2_Nov16Tourney extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatHW_Async robot = new CatHW_Async();  // Use our new mecanum async hardware


    // Our constructor for this class
    public Mec_TeleOpLevel2_Nov16Tourney() {
        robot = new CatHW_Async();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(robot.isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            robot.underLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            robot.underLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }


        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        elapsedTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean grabMode = true;

        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Drive train speed control:
            if (gamepad1.left_bumper) {
                driveSpeed = 1.0;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.25;
            } else {
                driveSpeed = 0.7;
            }

            // Input for setDrivePowers train and sets the dead-zones:
            leftFront  = -((Math.abs(gamepad1.right_stick_y) < 0.1) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.1) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightFront = -((Math.abs(gamepad1.right_stick_y) < 0.1) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.1) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;
            leftBack   = -((Math.abs(gamepad1.right_stick_y) < 0.1) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.1) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightBack  = -((Math.abs(gamepad1.right_stick_y) < 0.1) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.1) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;

            // Calculate the scale factor:
            SF = robot.driveClassic.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront  = leftFront  * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack   = leftBack   * SF * driveSpeed;
            rightBack  = rightBack  * SF * driveSpeed;
            // DRIVE!!!
            robot.driveClassic.setDrivePowers(leftFront, rightFront, leftBack, rightBack);

            // Jaws Control:
            robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger*0.3));

            // stonePusher controls:
            if (gamepad1.dpad_left) {
                robot.jaws.pusherRetract();
            }
            //if (gamepad1.dpad_left){
            //    robot.jaws.pusherMid();
            //}
            if (gamepad1.dpad_right){
                robot.jaws.pusherFullPush();
            }


            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */


            // Open/Close Foundation Fingers:
            if(gamepad2.y) {
                robot.claw.retractClaw();
            }
            if (gamepad2.x) {
                robot.claw.extendClaw();
            }

            // Tail/Stacker lift motor controls:
            robot.tail.tailLift.setPower(-gamepad2.right_stick_y);

            // Extend and wrist controls:
            robot.tail.tailExtend.setPower(gamepad2.left_stick_y);

            if (gamepad2.dpad_right){
                grabMode = true;
            }
            if (gamepad2.dpad_left){
                grabMode = false;
            }

            if(grabMode){
                robot.tail.wristServo.setPower(gamepad2.left_stick_x-.95);
            }else {
                robot.tail.wristServo.setPower(-gamepad2.left_stick_x+1);
            }


            // Thumb controls:
            if (gamepad2.left_bumper) {
                robot.tail.closeThumb();
            }
            if (gamepad2.right_bumper){
                robot.tail.openThumb();
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
            telemetry.addData("Intake Power:","%.2f", robot.jaws.leftJawMotor.getPower());

            telemetry.addData("Encoder lf/lr rf/rr", "%5d %5d   %5d %5d",
                    robot.driveClassic.leftFrontMotor.getCurrentPosition(),
                    robot.driveClassic.leftRearMotor.getCurrentPosition(),
                    robot.driveClassic.rightFrontMotor.getCurrentPosition(),
                    robot.driveClassic.rightRearMotor.getCurrentPosition()  );
            telemetry.update();
        }
    }
}
