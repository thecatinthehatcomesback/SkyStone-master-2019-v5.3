package org.firstinspires.ftc.teamcode;


/**
 * Written by Team #10273, The Cat in the Hat Comes Back.
 */
public class CurvePoint {

    // Variables:
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;


    /**
     *
     * @param x
     * @param y
     * @param moveSpeed
     * @param turnSpeed
     * @param followDistance
     * @param pointLength
     * @param slowDownTurnRadians
     * @param slowDownTurnAmount
     */
    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance,
                      double pointLength, double slowDownTurnRadians, double slowDownTurnAmount) {

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    /**
     *
     * @param thisPoint
     */
    public CurvePoint(CurvePoint thisPoint) {
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        pointLength = thisPoint.pointLength;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
    }

    /**
     *
     * @return
     */
    public Point toPoint() {
        return new Point(x, y);
    }

    /**
     *
     * @param point
     */
    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }
}
