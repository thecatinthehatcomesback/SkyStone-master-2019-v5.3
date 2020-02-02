package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;

/**
 * Created by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoAllUpdates implements Runnable{

    // static variable singleInstance of type Singleton
    private static CatOdoAllUpdates singleInstance = null;

    // Thread run condition
    private boolean isRunning   = true;
    private int     sleepTime   = 25;

    CatOdoPositionUpdate positionUpdate = null;
    CatOdoPowerUpdate powerUpdate = null;

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    private CatOdoAllUpdates(ExpansionHubEx inExpansionHub, DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH) {
        positionUpdate = new CatOdoPositionUpdate(inExpansionHub, verticalEncoderLeft, verticalEncoderRight, horizontalEncoder, COUNTS_PER_INCH);
        powerUpdate = new CatOdoPowerUpdate(positionUpdate);
        //positionUpdate.reverseLeftEncoder();
    }

    public static CatOdoAllUpdates getInstanceAndInit(ExpansionHubEx inExpansionHubIn, DcMotor verticalEncoderLeftIn, DcMotor verticalEncoderRightIn, DcMotor horizontalEncoderIn, double COUNTS_PER_INCHIn){
        if (singleInstance == null) {
            singleInstance = new CatOdoAllUpdates(inExpansionHubIn,verticalEncoderLeftIn,verticalEncoderRightIn,horizontalEncoderIn,COUNTS_PER_INCHIn);
        }
        return singleInstance;
    }

    public void resetThreads(){
        powerUpdate.reset();
        positionUpdate.resetPos();
        isRunning = true;
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            positionUpdate.globalCoordinatePositionUpdate();
            powerUpdate.updatePower();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
