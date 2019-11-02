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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.CatHW_DriveClassic.CHILL_SPEED;


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
        double intakeSpeed;
        boolean autoIntake = false;

        CatOdoPositionUpdate globalPositionUpdate = new CatOdoPositionUpdate(robot.driveOdo.leftOdometry, robot.driveOdo.rightOdometry, robot.driveOdo.backOdometry, robot.driveOdo.ODO_COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();


        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * ---   _________________   ---
             * ---   Driver 1 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */

            // Drive train speed control:
            if (gamepad1.left_bumper) {
                driveSpeed = 1;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.4;
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



            /**
             * ---   _________________   ---
             * ---   Driver 2 controls   ---
             * ---   \/ \/ \/ \/ \/ \/   ---
             */


            // Intake speed control:
            if (gamepad2.left_bumper) {
                intakeSpeed = 1;
            } else if (gamepad2.right_bumper) {
                intakeSpeed = 0.6;
            } else {
                intakeSpeed = 0.3;
            }

            if (gamepad2.right_stick_button) {
                autoIntake = true;
                robot.intake.runtime.reset();
                robot.intake.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intake.intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intake.intakeMotor.setPower(CHILL_SPEED);
                robot.intake.intakeMotor.setTargetPosition(-125);
            }
            if (autoIntake) {
                if (robot.intake.isDone()) {
                    autoIntake = false;
                    robot.intake.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
            else{
                // Control the spinning intake:
                robot.intake.intakeMotor.setPower(intakeSpeed * gamepad2.left_stick_y);
            }

            // Open/Close Foundation Fingers:
            if(gamepad2.y) {
                robot.tail.releaseFoundationFingers();
            } else if (gamepad2.x) {
                robot.tail.grabFoundationFingers();
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
            telemetry.addData("Intake Power:","%.2f", robot.intake.intakeMotor.getPower());

            telemetry.addData("X Position","%.2f", globalPositionUpdate.returnXCoordinate() / robot.driveOdo.ODO_COUNTS_PER_INCH);
            telemetry.addData("Y Position", "%.2f",globalPositionUpdate.returnYCoordinate() / robot.driveOdo.ODO_COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", "%.2f", globalPositionUpdate.returnOrientation());

            telemetry.addData("Intake Encoder:", robot.intake.intakeMotor.getCurrentPosition());
            telemetry.addData("left Encoder:", robot.driveOdo.leftOdometry.getCurrentPosition());
            telemetry.addData("right Encoder:", robot.driveOdo.rightOdometry.getCurrentPosition());
            telemetry.addData("back Encoder:", robot.driveOdo.backOdometry.getCurrentPosition());

            telemetry.update();
        }
        globalPositionUpdate.stop();
    }
}
