package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.ImprovedPurePursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelOdometry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.movementX;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.movementY;
//import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.movementTurn;
import static java.lang.Math.*;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 2019-12-05.
 */
public class PathGenerator {

    static int lastClosestPoint = 0;
    private UpdateThread updateThread;
    private DriveWheelOdometry odometry;
    private double COUNTS_PER_REV = 537.6;
    private double EXTERNAL_GEAR_RATIO = 0.78125;     // This is < 1.0 if geared UP
    private double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    private double CPI = ((COUNTS_PER_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415));
    private final double Kp = 0.0015;
    public static double scaledLeft = 0;
    public static double scaledRight = 0;

    public PathGenerator(Telemetry telemetry, HardwareMap hardwareMap) {
        odometry = new DriveWheelOdometry(hardwareMap, telemetry);
    }

    public void init() {
        lastClosestPoint = 0;
        scaledLeft = 0;
        scaledRight = 0;
    }
    public void startThread() {
        Thread updater = new Thread(updateThread);
        updater.start();
    }

    public void stopThread() {
        updateThread.stop();
    }

    public void injectPoints(ArrayList<Point> pathPoints, double spacing) {

        ArrayList<Point> newPoints = new ArrayList<>();
        for (int i=0; i<pathPoints.size()-1; i++) {

            Point startPoint = pathPoints.get(i);//(0,0)
            Point endPoint = pathPoints.get(i+1);//(10,10)

            double magnitude = hypot(endPoint.x - startPoint.x, endPoint.y-startPoint.y);//~14

            double pointsThatFit = ceil(magnitude / spacing);//3

//            Point normalized = new Point(endPoint.x/magnitude, endPoint.y/magnitude);//0.7,0.7

            Point vector = new Point(endPoint.x-startPoint.x,endPoint.y-startPoint.y);//10,10

            Point normalizedVector = new Point(vector.x/magnitude,vector.y/magnitude);//~0.7,0.7

            normalizedVector.x *= (magnitude / pointsThatFit);//~3.3
            normalizedVector.y *= (magnitude / pointsThatFit);

            for (int j=0; j<pointsThatFit; j++) {//0,0-----0.7,0.7------1.4,1.4
//                pathPoints.add(new Point(startPoint.x+normalized.x*j,startPoint.y+normalized.y*j));
                //0,0----3.3,3.3----6.6,6.6 if keep going 9.9, 9.9 Oh well more points means more accuracy
                newPoints.add(new Point(startPoint.x+normalizedVector.x*j,startPoint.y+normalizedVector.y*j));
            }
            newPoints.add(endPoint);
        }
        for (int n=0; n<=newPoints.size()-1; n++) {
            pathPoints.add(newPoints.get(n));
        }
    }

    /**
     *
     * @param pathPoints
     * @param b Weight smooth which should be between 0.75-0.98
     */
    public void smoothPath(ArrayList<Point> pathPoints, double b) {
        ArrayList<Point> newPath = new ArrayList<>(pathPoints);

        double tolerance = 0.001;//Higher means less smoothing
        double a = 1-b;

        double change = tolerance;

        while (change >= tolerance) {
            change = 0.0;

            for (int i=1; i<pathPoints.size()-1; i++) {
                Point oldPoint = pathPoints.get(i);
                Point currentPoint = pathPoints.get(i);
                Point previousPoint = pathPoints.get(i-1);
                Point nextPoint = pathPoints.get(i+1);
                Point currentPointCopy = new Point(currentPoint.x,currentPoint.y);

                currentPoint.x += a * (oldPoint.x - currentPoint.x) + b * (previousPoint.x + nextPoint.x - 2 * currentPoint.x);
                currentPoint.y += a * (oldPoint.y - currentPoint.y) + b * (previousPoint.y + nextPoint.y - 2 * currentPoint.y);

                change += abs(currentPointCopy.x - currentPoint.x);
                change += abs(currentPointCopy.y - currentPoint.y);
            }
        }
        for (Point points : newPath) {
            pathPoints.add(new Point(points.x,points.y));
        }
    }

    /**
     * Gets the curvature at a certain point
     * @param pathPoints
     * @param pointIndex Index of the point to get the curvature at
     * @return
     */
    public double getCurvature(ArrayList<Point> pathPoints, int pointIndex) {

        Point P = pathPoints.get(pointIndex-1);//x1,y1
        Point Q = pathPoints.get(pointIndex);//x2,y2
        Point R = pathPoints.get(pointIndex+1);//x3,y3

        double x1 = P.x;
        double y1 = P.y;
        double x2 = Q.x;
        double y2 = Q.y;
        double x3 = R.x;
        double y3 = R.y;

        if (x1 == x2) {//No divide by 0 errors
            x1 += 0.001;
        }

        double k1 = 0.5 * (pow(x1,2) + pow(y1,2) - pow(x2,2) - pow(y2,2)) / (x1 - x2);
        double k2 = (y1 - y2) / (x1 - x2);
        double b = 0.5 * (pow(x2,2) - 2 * x2 * k1 + pow(y2,2) - pow(x3,2) + 2 * x3 * k1 - pow(y3,2)) / (x3 * k2 - y3 + y2 - x2 * k2);
        double a = k1 - k2 * b;
        double r = sqrt(pow(x1 - a,2) + pow(y1 - b,2));

        return 1/r;//curvature
    }

    /**
     *
     * @param pathPoints
     * @param pointIndex The index of the point to get the velocity at.
     * @param maxVelocity The paths maximum velocity
     * @param maxAcceleration The maximum acceleration
     * @param k Constant around 1-5 based on how fast you want the robot to go around turns.
     * @return
     */
    public double getPointVelocity(ArrayList<Point> pathPoints, int pointIndex, double maxVelocity, double maxAcceleration, double k) {

        double currentPointVeloctiy = min(maxVelocity, k/getCurvature(pathPoints,pointIndex));
        double previousPointVeloctiy = min(maxVelocity, k/getCurvature(pathPoints,pointIndex-1));
        double nextPointVeloctiy = min(maxVelocity, k/getCurvature(pathPoints,pointIndex+1));
        double d = distanceFormula(pathPoints.get(pointIndex).x - pathPoints.get(pointIndex-1).x, pathPoints.get(pointIndex).y - pathPoints.get(pointIndex-1).y);
        double a = maxAcceleration;
        double maximumVelocityAtPoint = sqrt(pow(previousPointVeloctiy,2) + 2 * a * d);

        //Now get the new target velocity
        previousPointVeloctiy = 0;
        d = distanceFormula(pathPoints.get(pointIndex+1).x-pathPoints.get(pointIndex).x,pathPoints.get(pointIndex+1).y - pathPoints.get(pointIndex).y);
        double newVelocity = min(currentPointVeloctiy,sqrt(pow(nextPointVeloctiy,2) + 2 * a * d));

        return newVelocity;
    }

    public int getClosetPointIndex(ArrayList<Point> pathPoints, Point robotPose) {
        double shortestDistance = Double.MAX_VALUE;
        int closestPoint = 0;

        for (int i=lastClosestPoint; i<=pathPoints.size()-1; i++) {
            if (distanceFormula(pathPoints.get(i).x*CPI-robotPose.x, pathPoints.get(i).y*CPI-robotPose.y) < shortestDistance) {
                closestPoint = i;
                shortestDistance = distanceFormula(pathPoints.get(i).x*CPI-robotPose.x, pathPoints.get(i).y*CPI-robotPose.y);
            }
        }
        lastClosestPoint = closestPoint;
        return closestPoint;
    }

    public void followPath(ArrayList<Point> pathPoints, double speed, double lookAheadDistance) {

        int closestPointIndex = getClosetPointIndex(pathPoints, new Point(odometry.xLocation, odometry.yLocation));

        Point E = pathPoints.get(closestPointIndex);
        Point L = pathPoints.get(closestPointIndex+1);
        Point C = new Point(odometry.xLocation, odometry.yLocation);
        double r = lookAheadDistance;
        Point d = new Point(L.x-E.x,L.y-E.y);
        Point f = new Point(E.x-C.x, E.y-C.y);

        double a = d.dot(d);
        double b = 2*f.dot(d);
        double c = f.dot(f) - r*r;
        double discriminant = b*b - 4*a*c;
        double tValue = Double.NaN;

        if (discriminant < 0) {
            //No intersection
        } else {
            discriminant = sqrt(discriminant);
            double t1 = (-b - discriminant)/(2*a);
            double t2 = (-b + discriminant)/(2*a);

            if (t1 >= 0 && t1 <= 1) {
                //return t1 intersections
                tValue = t1;
            }
            if (t2 >= 0 && t2 <= 1) {
                //return t2 intersections
                tValue = t2;
            }
            //Otherwise don't return any intersections.
        }

        Point intersectionPoint = new Point(E.x+tValue*d.x,E.y+tValue*d.y);

        double robotA = -tan(odometry.getRawHeading());
        double robotB = 1;
        double robotC = tan(odometry.getRawHeading())*odometry.xLocation-odometry.yLocation;

        double x = abs(robotA*intersectionPoint.x+robotB*intersectionPoint.y+robotC) / sqrt(pow(robotA,2)+pow(robotB,2));
        double curvature = (2*x) / pow(lookAheadDistance,2);

        double side = signum(sin(odometry.getRawHeading())*(intersectionPoint.x-odometry.xLocation)-cos(odometry.getRawHeading())*(intersectionPoint.y-odometry.yLocation));

        double signedCurvature = curvature * side;

        double V = getPointVelocity(pathPoints,closestPointIndex,0.8, 0.2,3);
        double T = 18;//Track width of 18 inches
        double leftVelocity = V * (2 + signedCurvature*T)/2;
        double rightVelocity = V * (2 - signedCurvature*T)/2;

        scaledLeft = leftVelocity * speed;
        scaledRight = rightVelocity * speed;

        if (closestPointIndex == pathPoints.size()-1) {
            scaledLeft = 0;
            scaledRight = 0;
        }

    }


    /**
     * The distance formula.
     * @param x
     * @param y
     * @return
     */
    public double distanceFormula(double x, double y) {
        return hypot(x, y);
    }
}