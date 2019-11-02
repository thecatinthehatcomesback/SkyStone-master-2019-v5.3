/*
        CatThreadAsync.java

    An helper class containing code to be used when a thread is needed that
    keeps polling the waitUntilDone() method of a subsystem.

*/

package org.firstinspires.ftc.teamcode;

/**
 * This is NOT an OpMode.
 *
 * This class is used to spawn a wait for a subsystem to be done action
 *
 */
public class CatThreadAsync extends Thread {
    CatHW_Subsystem subsystem;

    /* Constructor */
    public CatThreadAsync(CatHW_Subsystem newSubsystem){
        subsystem = newSubsystem;
    }


    public void run() {
        subsystem.waitUntilDone();
    }
}// End of class bracket