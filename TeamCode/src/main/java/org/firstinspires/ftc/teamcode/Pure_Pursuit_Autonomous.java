package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        robot.init(hardwareMap, this);
        FtcDashboard dashboard = FtcDashboard.getInstance();


        /*
        Init Delay Option Select:
         */
        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            // Leave the loop if STOP is pressed
            if (this.isStopRequested()) break;

            /*
            We don't need a "waitForStart()" since we've been running our own loop all this time so
            that we can make some changes.
             */
        }
        /*
        Runs after hit start:
        DO STUFF FOR the OpMode!!!
         */
        // Init the IMU after PLAY so that it doesn't drift.
        //robot.drive.IMU_Init();


        /* Go! */
        ArrayList<CurvePoint> simpleDrivePath = new ArrayList<>();

        simpleDrivePath.add(new CurvePoint(0, 0,0));
        simpleDrivePath.add(new CurvePoint(0, 100, 0));
        simpleDrivePath.add(new CurvePoint(72, 100, 90));
        simpleDrivePath.add(new CurvePoint(72, 10, 180));

        robot.drive.translateDrive(simpleDrivePath, .7, 0, 20.0, 16);
        robot.drive.waitUntilDone();


        simpleDrivePath.clear();
        simpleDrivePath.add(new CurvePoint(72, 10, 180));
        simpleDrivePath.add(new CurvePoint(0, 0,270));
        simpleDrivePath.add(new CurvePoint(0, 100, 360));
        simpleDrivePath.add(new CurvePoint(72, 100, 360 + 90));

        robot.drive.translateDrive(simpleDrivePath, .7, 0, 20.0, 16);
        robot.drive.waitUntilDone();

        simpleDrivePath.clear();
        simpleDrivePath.add(new CurvePoint(72, 100, 360 + 90));
        simpleDrivePath.add(new CurvePoint(72, 10, 360 + 180));
        simpleDrivePath.add(new CurvePoint(0, 0,360 + 270));
        simpleDrivePath.add(new CurvePoint(0, 100, 720));

        robot.drive.translateDrive(simpleDrivePath, .7, 0, 20.0, 16);
        robot.drive.waitUntilDone();

        simpleDrivePath.clear();
        simpleDrivePath.add(new CurvePoint(0, 100, 720));
        simpleDrivePath.add(new CurvePoint(72, 100, 720 + 90));
        simpleDrivePath.add(new CurvePoint(72, 10, 720 + 180));
        simpleDrivePath.add(new CurvePoint(20, 0,720 + 270));
        simpleDrivePath.add(new CurvePoint(0, 0,720 + 360));


        robot.drive.translateDrive(simpleDrivePath, .7, 0, 20.0, 16);
        robot.drive.waitUntilDone();


        robot.drive.updatesThread.stop();
    }
}