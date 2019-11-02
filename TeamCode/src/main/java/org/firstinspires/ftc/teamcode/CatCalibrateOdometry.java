package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Cat Odometry Calibrate", group = "Calibration")
public class CatCalibrateOdometry extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatAsyncHW robot = new CatAsyncHW();  // Use our new mecanum async hardware





    final double PIVOT_SPEED = -0.27;

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
// Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this);
        robot.drive.IMUinit();

        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(robot.drive.getCurrentAngle() < 90 && opModeIsActive()){
            if(robot.drive.getCurrentAngle() < 60) {
                robot.drive.drive(PIVOT_SPEED,-PIVOT_SPEED,PIVOT_SPEED,-PIVOT_SPEED);

            }else{
                robot.drive.drive(PIVOT_SPEED/2,-PIVOT_SPEED/2,PIVOT_SPEED/2,-PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", robot.drive.getCurrentAngle());
            telemetry.update();
        }

        //Stop the robot
        robot.drive.drive(0, 0, 0, 0);
        robot.robotWait(1.0);

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = robot.drive.getCurrentAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.drive.leftOdometry.getCurrentPosition()) + (Math.abs(robot.drive.rightOdometry.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*angle*verticalEncoderTickOffsetPerDegree)/(Math.PI*robot.drive.ODO_COUNTS_PER_INCH);

        // Negated this numberto move the robot center to the actual center instead of behind it
        horizontalTickOffset = -robot.drive.backOdometry.getCurrentPosition()/Math.toRadians(robot.drive.getCurrentAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", robot.drive.getCurrentAngle());
            telemetry.addData("Vertical Left Position", -robot.drive.leftOdometry.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.drive.rightOdometry.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.drive.backOdometry.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }




}
