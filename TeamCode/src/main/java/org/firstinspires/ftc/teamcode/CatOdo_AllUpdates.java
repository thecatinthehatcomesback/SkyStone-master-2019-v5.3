package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;

/**
 * CatOdoAllUpdates.java
 *
 *
 * This class spawns a separate thread to keep track the robot encoders using USB bulk reads to get
 * consistent odometry readings which helps for position and power calculations.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatOdo_AllUpdates implements Runnable
{
    /** static variable singleInstance of type Singleton. */
    private static CatOdo_AllUpdates singleInstance = null;

    /** Thread run condition. */
    private boolean isRunning = true;
    /** The amount of time the thread will rest before running its tasks again. */
    private int sleepTime = 25;

    /* Classes that will run off this thread. */
    CatOdo_PositionUpdate positionUpdate;
    CatOdo_PowerUpdate powerUpdate;

    /**
     * Stops the position update thread.
     */
    public void stop(){ isRunning = false; }

    /* Constructor */
    public CatOdo_AllUpdates(ExpansionHubEx inExpansionHub, DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH) {
        positionUpdate = new CatOdo_PositionUpdate(inExpansionHub, verticalEncoderLeft, verticalEncoderRight, horizontalEncoder, COUNTS_PER_INCH);
        powerUpdate = new CatOdo_PowerUpdate(positionUpdate);
        //positionUpdate.reverseLeftEncoder();
    }

    /**
     * TODO:  Add Javadoc.
     *
     * @param inExpansionHubIn
     * @param verticalEncoderLeftIn
     * @param verticalEncoderRightIn
     * @param horizontalEncoderIn
     * @param COUNTS_PER_INCHIn
     * @return
     */
    public static CatOdo_AllUpdates getInstanceAndInit(ExpansionHubEx inExpansionHubIn,
                                                       DcMotor verticalEncoderLeftIn,
                                                       DcMotor verticalEncoderRightIn,
                                                       DcMotor horizontalEncoderIn,
                                                       double COUNTS_PER_INCHIn) {
        if (singleInstance == null) {
            singleInstance = new CatOdo_AllUpdates(inExpansionHubIn, verticalEncoderLeftIn,
                    verticalEncoderRightIn,horizontalEncoderIn,COUNTS_PER_INCHIn);
        }

        return singleInstance;
    }

    /**
     * Resets the minimum power and position for the robot.
     */
    public void resetThreads() {
        powerUpdate.resetPowerToNormal();
        positionUpdate.resetPos();
        isRunning = true;
    }

    /**
     * Runs the thread.
     */
    @Override
    public void run() {
        while(isRunning) {
            positionUpdate.globalCoordinatePositionUpdate();
            //powerUpdate.updatePower();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}