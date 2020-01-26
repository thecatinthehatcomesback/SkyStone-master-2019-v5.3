/*
        Test_TeleOp.java

    A Linear opMode class to be our TeleOp testing method to try
    and solve our problems throughout the year without having to
    modify the main TeleOp.


    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Test Tele", group="CatTest TeleOp")
public class Test_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime elapsedGameTime = new ElapsedTime();

    //CatOdoPositionUpdate globalPositionUpdate;


    /* Declare OpMode members. */
    CatHW_Async robot;

    /* constructor for class */
    public Test_TeleOp() {
        robot = new CatHW_Async();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this, true);
        robot.driveClassic.IMU_Init();


        // Finished!  Now tell the driver...
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for PLAY:
        waitForStart();

        if(robot.isRedAlliance) {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
        }
        // Go!
        runTime.reset();
        elapsedGameTime.reset();

//        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.driveOdo.leftOdometry, robot.driveOdo.rightOdometry, robot.driveOdo.backOdometry, CatHW_DriveOdo.ODO_COUNTS_PER_INCH, 75);
//        Thread positionThread = new Thread(globalPositionUpdate);
//        positionThread.start();

        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;

        // Run until the end of the match (driver presses STOP)
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
            leftFront  = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;
            leftBack   = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightBack  = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
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




            // Tell us the odometry encoder ticks
            telemetry.addData("OdoTicks", "L/R/B  :%7d  :%7d  :%7d",
                    robot.driveOdo.updatesThread.positionUpdate.returnVerticalLeftEncoderPosition(),
                    robot.driveOdo.updatesThread.positionUpdate.returnVerticalRightEncoderPosition(),
                    robot.driveOdo.updatesThread.positionUpdate.returnNormalEncoderPosition() );
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
            telemetry.addData("Y Position", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
            telemetry.addData("Orientation (Degrees)", robot.driveOdo.updatesThread.positionUpdate.returnOrientation());

            telemetry.update();
        }

        //Stop the thread
        robot.driveOdo.updatesThread.stop();

    }
}
