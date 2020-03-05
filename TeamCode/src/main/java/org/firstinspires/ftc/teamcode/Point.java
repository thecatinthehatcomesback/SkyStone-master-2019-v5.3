package org.firstinspires.ftc.teamcode;


/**
 * @author Team #10273, The Cat in the Hat Comes Back.
 */
public class Point {

    // Variables:
    public double x;
    public double y;

    /**
     * Create a point and angle for coordinates and use in Pure Pursuit.
     *
     * @param x X-point on a cartesian coordinate plane.
     * @param y Y-point on a cartesian coordinate plane.
     */
    Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public CurvePoint toCurvePoint() {
        return new CurvePoint(this.x, this.y, 0);
    }
}
