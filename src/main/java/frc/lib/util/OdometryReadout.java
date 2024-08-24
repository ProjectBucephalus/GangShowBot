package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OdometryReadout 
{
    public double[] XPoints;
    public double[] YPoints;
    private double robotLength;

    public OdometryReadout(double robotLength)
    {
        this.robotLength = robotLength;
    //    addPoint(robotLength/2,0);
        addPoint(robotLength,0);
        addPoint(robotLength/2,robotLength/2);
        addPoint(0,robotLength);
        addPoint(-robotLength/2,robotLength/2);
        addPoint(-robotLength,0);
        addPoint(-robotLength/2,-robotLength/2);
        addPoint(0,-robotLength);
        addPoint(robotLength/2,-robotLength/2);
    }

    public Translation2d getPoint(int index)
    {
        return new Translation2d(XPoints[index],YPoints[index]);
    }

    public void addPoint(double x, double y)
    {
        XPoints[XPoints.length] = x;
        YPoints[YPoints.length] = y;
    }

    public void addPoint(Translation2d xy)
    {
        addPoint(xy.getX(),xy.getY());
    }

    public void editPoint(double x, double y, int index)
    {
        XPoints[index] = x;
        YPoints[index] = y;
    }

    public void editPoint(Translation2d xy, int index)
    {
        editPoint(xy.getX(),xy.getY(),index);
    }

    public void addRectangle(double x1, double y1, double x2, double y2)
    {
        int nX = 2 * (int) Math.ceil(Math.abs(x1 - x2));
        int nY = 2 * (int) Math.ceil(Math.abs(y1 - y2));

        addPoint(x1,y1);
        addPoint(x1,y2);
        addPoint(x2,y1);
        addPoint(x2,y2);
        for (int i = 1; i < nX-1; i++)
        {
            addPoint(x1 + i*((x2-x1)/nX), y1);
            addPoint(x1 + i*((x2-x1)/nX), y2);
        }
        for (int i = 1; i < nY-1; i++)
        {
            addPoint(x1, y1 + i*((y2-y1)/nY));
            addPoint(x2, y1 + i*((y2-y1)/nY));
        }
    }

    public void addRectangle(Translation2d xy1, Translation2d xy2)
    {
        addRectangle(xy1.getX(), xy1.getY(), xy2.getX(), xy2.getY());
    }

    public void addRectangle(Translation2d[] corners)
    {
        addRectangle(corners[0], corners[1]);
    }

    public void robotPose(Translation2d robotCentre)
    {
        double rX = robotCentre.getX();
        double rY = robotCentre.getY();
        editPoint(robotCentre,0);
        editPoint(rX + robotLength, rY + 0, 1);
        editPoint(rX + robotLength/2, rY + robotLength/2, 2);
        editPoint(rX + 0, rY + robotLength, 3);
        editPoint(rX + -robotLength/2, rY + robotLength/2, 4);
        editPoint(rX + -robotLength, rY + 0, 5);
        editPoint(rX + -robotLength/2, rY + -robotLength/2, 6);
        editPoint(rX + 0, rY + -robotLength, 7);
        editPoint(rX + robotLength/2, rY + -robotLength/2, 8);
    }
}
