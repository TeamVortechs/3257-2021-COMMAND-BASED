package frc.robot.utils;

import java.awt.geom.Point2D;


public class BezierCurve 
{
    Point2D endPoint, control1, control2;
    double length = 0;

    public BezierCurve(Point2D _control1, Point2D _control2, Point2D _endPoint) 
    {
        control1 = new Point2D.Double(_control1.getX(), _control1.getY());
        control2 = new Point2D.Double(_control2.getX(), _control2.getY());
        endPoint = new Point2D.Double(_endPoint.getX(), _endPoint.getY());

        int samples = 200;
        double[] arcLengths = new double[samples + 1];
        arcLengths[0] = 0;

        double ox = getPos(0).getX();
        double oy = getPos(0).getY();

        for (int i = 1; i <= samples; i += 1) 
        {
            double x = getPos(i * 0.001).getX();
            double y = getPos(i * 0.001).getY();
            double dx = ox - x, dy = oy - y;
            length += Math.sqrt(dx * dx + dy * dy);
            arcLengths[i] = length;
            ox = x;
            oy = y;
        }
    }

    public Point2D getPos(double t) {
        return new Point2D.Double(
                (1 - t) * ((1 - t) * ((1 - t) * 0 + t * control1.getX())
                        + t * ((1 - t) * control1.getX() + t * control2.getX()))
                        + t * ((1 - t) * ((1 - t) * control1.getX() + t * control2.getX())
                                + t * ((1 - t) * control2.getX() + t * endPoint.getX())),
                (1 - t) * ((1 - t) * ((1 - t) * 0 + t * control1.getY())
                        + t * ((1 - t) * control1.getY() + t * control2.getY()))
                        + t * ((1 - t) * ((1 - t) * control1.getY() + t * control2.getY())
                                + t * ((1 - t) * control2.getY() + t * endPoint.getY())));
    }

    public double getDX(double t) {
        return Math.pow(3 * (1 - t), 2) * (control1.getX() - 0) + 6 * (1 - t) * t * (control2.getX() - control1.getX())
                + 3 * Math.pow(t, 2) * (endPoint.getX() - control2.getX());
    }

    public double getDY(double t) {
        return Math.pow(3 * (1 - t), 2) * (control1.getY() - 0) + 6 * (1 - t) * t * (control2.getY() - control1.getY())
                + 3 * Math.pow(t, 2) * (endPoint.getY() - control2.getY());
    }

    public double getAngle(double t) 
    {
        return Math.toDegrees(Math.atan2(getDY(t), getDX(t)));
    }

    public double getLength() { return length; }
}