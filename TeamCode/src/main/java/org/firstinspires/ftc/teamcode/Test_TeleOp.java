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

    /* Declare OpMode members. */
    CatHW_Async robot;

    boolean inReverse = true;

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


        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Tell us the odometry encoder ticks
            telemetry.addData("OdoTicks", "L/R/B  :%7d  :%7d  :%7d",
                    robot.driveOdo.leftOdometry.getCurrentPosition(),
                    robot.driveOdo.rightOdometry.getCurrentPosition(),
                    robot.driveOdo.backOdometry.getCurrentPosition());
            telemetry.update();
        }
    }
}
