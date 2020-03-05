package org.firstinspires.ftc.teamcode;


/**
 * @author Team #10273, The Cat in the Hat Comes Back.
 */
public class CurvePoint extends Point {

    // Attributes/Fields: //
    public double followDistance;
    //public int theta;
    //public double pointLength;
    //public double slowDownTurnRadians;
    //public double slowDownTurnAmount;


    /**
     * Constructor which takes in double values to build CurvePoint.
     *
     * @param x coordinate in inches.
     * @param y coordinate in inches.
     * @param followDistance for intersection distance in inches.
     */
    public CurvePoint(double x, double y, double followDistance) {
        super(x, y);
        this.followDistance = followDistance;
    }

    /**
     * Constructor that takes in a CurvePoint to build CurvePoint.
     *
     * @param thisPoint to set CurvePoint to.
     */
    public CurvePoint(CurvePoint thisPoint) {
        super(thisPoint.x, thisPoint.y);
        this.followDistance = thisPoint.followDistance;
    }

    /**
     * Sets the Point values for a CurvePoint.
     *
     * @param point to set the values to.
     */
    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }
}
