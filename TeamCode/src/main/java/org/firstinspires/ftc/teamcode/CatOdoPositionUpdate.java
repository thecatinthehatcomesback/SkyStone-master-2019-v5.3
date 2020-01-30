package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
/**
 * Original created by Sarthak (of Wizards.exe) on 6/1/2019.
 * Modified by Team #10273, The Cat in the Hat Comes Back.
 */
public class CatOdoPositionUpdate {
    //Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;
    private ExpansionHubEx expansionHub;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftEncoderPositionMultiplier = -1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = -1;
    private int leftEncoderValue = 0;
    private int rightEncoderValue = 0;
    private int horizontalEncoderValue = 0;
    private RevBulkData bulkData;

    private double count_per_in;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     */
    public CatOdoPositionUpdate(ExpansionHubEx inExpansionHub, DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH){
        count_per_in = COUNTS_PER_INCH;
        expansionHub = inExpansionHub;
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    public CatOdoPositionUpdate(ExpansionHubEx inExpansionHub, DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH, double startingX,
                                double startingY, double startingOrientation){
        count_per_in = COUNTS_PER_INCH;
        expansionHub = inExpansionHub;
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;

        this.robotGlobalXCoordinatePosition = startingX;
        this.robotGlobalYCoordinatePosition = startingY;
        this.robotOrientationRadians = Math.toRadians(startingOrientation);

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    public void globalCoordinatePositionUpdate(){
        //do a bulk read of encoders
        bulkData = expansionHub.getBulkInputData();
        leftEncoderValue = bulkData.getMotorCurrentPosition(verticalEncoderLeft);
        rightEncoderValue = bulkData.getMotorCurrentPosition(rightEncoderValue);
        horizontalEncoderValue = bulkData.getMotorCurrentPosition(horizontalEncoder);
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (leftEncoderValue * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (rightEncoderValue * verticalRightEncoderPositionMultiplier);

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        //Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
        double robotOrientationRadiansHalf = robotOrientationRadians - (changeInRobotOrientation / 2);

        //Get the components of the motion
        normalEncoderWheelPosition = (horizontalEncoderValue*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadiansHalf) + n*Math.cos(robotOrientationRadiansHalf));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadiansHalf) - n*Math.sin(robotOrientationRadiansHalf));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;

        Log.d("catbot", String.format("OdoTicks L/R/B  :%7d  :%7d  :%7d   X: %5.2f  Y: %5.2f  theta: %5.2f",
                returnVerticalLeftEncoderPosition(),
                returnVerticalRightEncoderPosition(),
                returnNormalEncoderPosition(),
                returnXInches(),
                returnYInches(),
                returnOrientation()));
    }

    public double returnXInches(){ return robotGlobalXCoordinatePosition/count_per_in; }

    public double returnYInches(){ return robotGlobalYCoordinatePosition/count_per_in; }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians); }

    public int returnVerticalLeftEncoderPosition(){
        return (leftEncoderValue * verticalLeftEncoderPositionMultiplier);
    }

    public int returnVerticalRightEncoderPosition(){
        return (rightEncoderValue * verticalRightEncoderPositionMultiplier);
    }

    public int returnNormalEncoderPosition(){
        return (horizontalEncoderValue * normalEncoderPositionMultiplier);
    }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    public void setAngleOffset(double offset){
        robotOrientationRadians += Math.toRadians(offset);
    }

    public void setRobotGlobalCoordinate(double x, double y, double theta){
        robotGlobalXCoordinatePosition = x;
        robotGlobalYCoordinatePosition = y;
        robotOrientationRadians = Math.toRadians(theta);
    }

}
