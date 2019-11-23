/*
        Mec_TeleOpLevel1_Ri2W.java

    A Linear opmode class that is used as our
    TeleOp method for the driver controlled period.

    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="Motor Test TeleOp", group="CatTest TeleOp")
public class Test_MotorTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatHW_Async robot = new CatHW_Async();  // Use our new mecanum async hardware

    // Our constructor for this class
    public Test_MotorTeleOp() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this, false);
        // Define and Initialize Motors //

        // Set Motor and Servo Modes //

        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        elapsedTime.reset();

        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            //telemetry.addData("Test Power:", "%.2f", testMotor.getPower());
            //telemetry.addData("Test Encoder:", "%d", testMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
