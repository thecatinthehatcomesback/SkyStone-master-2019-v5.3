package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoAllUpdates implements Runnable{
    // Thread run condition
    private boolean isRunning   = true;
    private int     sleepTime   = 25;

    CatOdoPositionUpdate positionUpdate = null;
    CatOdoPowerUpdate powerUpdate = null;

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public CatOdoAllUpdates(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH) {
        positionUpdate = new CatOdoPositionUpdate(verticalEncoderLeft, verticalEncoderRight, horizontalEncoder, COUNTS_PER_INCH);
        powerUpdate = new CatOdoPowerUpdate(positionUpdate);
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
