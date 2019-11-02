/*
        MecTeleOpLevel1_Ri2W.java

    A Linear opmode class that is used as our
    TeleOp method for the driver controlled period.

    This file is a modified version from the FTC SDK.
    Modifications by FTC Team #10273, The Cat in the Hat Comes Back.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Motor Test TeleOp", group="CatTeleOp")
public class TeleOpMotorTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware
    DcMotor testMotor = null;

    // Our constructor for this class
    public TeleOpMotorTest() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        // Define and Initialize Motors //
        testMotor = hardwareMap.dcMotor.get("test_motor");

        // Set Motor and Servo Modes //
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            testMotor.setPower(gamepad1.right_trigger);



            /**
             * ---   _________   ---
             * ---   TELEMETRY   ---
             * ---   \/ \/ \/    ---
             */
            telemetry.addData("Test Power:", "%.2f", testMotor.getPower());
            telemetry.addData("Test Encoder:", "%d", testMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
