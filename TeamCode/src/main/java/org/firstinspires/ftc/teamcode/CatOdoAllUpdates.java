package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Modified by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoAllUpdates implements Runnable{
    //Thead run condition
    private boolean isRunning   = true;
    private int sleepTime    = 0;

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }


    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            //globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
