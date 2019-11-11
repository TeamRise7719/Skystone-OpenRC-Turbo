package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.Math.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMath.lineCircleIntersection;

/**
 * Created by Sean Cardosi.
 * PurePursuitMovement is a class containing all of the functions to find and
 * provide movement to the drivetrain while following a set of given points.
 *
 * EVERYTHING IS CURRENTLY IN CENTIMETERS!!! ALL GRYO ANGLES ARE IN RADIANS!!!
 */
public class PurePursuitMovement {

    static Telemetry telemetry;

    static BNO055IMU gyro;
    static Orientation angles;

    static double movementX = 0.0;
    static double movementY = 0.0;
    static double movementTurn = 0.0;

    private static PurePursuitDrivetrain pwr;
    private static Odometry odometry;

    public PurePursuitMovement(Telemetry tel, HardwareMap hardwareMap) {
        telemetry = tel;

        pwr = new PurePursuitDrivetrain(hardwareMap);
        odometry = new Odometry(hardwareMap, tel);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        gyro                             = hardwareMap.get(BNO055IMU.class, "imuINT");

        gyro.initialize(parameters);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

//    private static CurvePoint extendLine(CurvePoint firstPoint, CurvePoint secondPoint, double distance) {
//
//        /**
//         * Since we are pointing to this point, extend the line if it is the last line
//         * but do nothing if it isn't the last line
//         *
//         * So if you imagine the robot is almost done its path, without this algorithm
//         * it will just point to the last point on its path creating craziness around
//         * the end (although this is covered by some sanity checks later).
//         * With this, it will imagine the line extends further and point to a location
//         * outside the endpoint of the line only if it's the last point. This makes the
//         * last part a lot smoother, almost looking like a curve but not.
//         */
//
//        //get the angle of this line
//        double lineAngle = Math.atan2(secondPoint.y - firstPoint.y,secondPoint.x - firstPoint.x);
//        //get this line's length
//        double lineLength = Math.hypot(secondPoint.x - firstPoint.x,secondPoint.y - firstPoint.y);
//        //extend the line by 1.5 pointLengths so that we can still point to it when we
//        //are at the end
//        double extendedLineLength = lineLength + distance;
//
//        CurvePoint extended = new CurvePoint(secondPoint);
//        extended.x = Math.cos(lineAngle) * extendedLineLength + firstPoint.x;
//        extended.y = Math.sin(lineAngle) * extendedLineLength + firstPoint.y;
//        return extended;
//    }


    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

//        //This should allow you to extend the final point. Should just be able to change "allPoints" in followMe to pathExtended
//        ArrayList<CurvePoint> pathExtended = (ArrayList<CurvePoint>) allPoints.clone();

        //Change                                 this \/         to pathExtended
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(odometry.xLocation,odometry.yLocation), allPoints.get(0).followDistance);

//        //This is the math that actually extends the final point
//        pathExtended.set(pathExtended.size()-1, extendLine(allPoints.get(allPoints.size()-2),allPoints.get(allPoints.size()-1), allPoints.get(allPoints.size()-1).pointLength * 1.5));

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

        telemetry.addData("FollowMePoint", "(%f,%f)", followMe.x, followMe.y);

    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {//NOTE: I took out xPos and yPos. I did not use them.

        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i=0; i<pathPoints.size() - 1; i++) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;

            for (Point thisIntersection : intersections) {

                double angle = Math.atan2(thisIntersection.y - odometry.yLocation, thisIntersection.x - odometry.xLocation);
                double deltaAngle = Math.abs(PurePursuitMath.AngleWrap(angle - angles.firstAngle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }


    /**
     * Basic run to a position. Better to use followCurve as it implements this and uses it better.
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x - odometry.xLocation, y - odometry.yLocation);

        double absoluteAngleToTarget = Math.atan2(y - odometry.yLocation, x - odometry.xLocation);//WorldPosition is the robots starting position and should be updated to the robots current position throughout OpMode

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (angles.firstAngle - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint * distanceToTarget);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementX = movementXPower * movementSpeed;//Movement X and Y are the power to apply to the motors
        movementY = movementYPower * movementSpeed;

        pwr.ApplyPower();//This "should" apply power to the robot correctly. It should work.
        odometry.updateLocation();//This "should" update the robots x and y locations

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1,1) * turnSpeed;//movement Turn is power to turn angles

        if (distanceToTarget < 10) {// < 10cm
            movementTurn = 0;
        }
    }
}
