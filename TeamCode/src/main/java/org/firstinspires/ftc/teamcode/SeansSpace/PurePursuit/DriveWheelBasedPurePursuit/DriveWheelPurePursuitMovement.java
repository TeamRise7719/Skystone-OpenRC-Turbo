package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitMathElements.CurvePoint;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitMathElements.PurePursuitMath;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitMathElements.PurePursuitMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitMathElements.PurePursuitMath.lineCircleIntersection;

/**
 * Created by Sean Cardosi.
 * PurePursuitMovement is a class containing all of the functions to find and
 * provide movement to the drivetrain while following a set of given points.
 */
public class DriveWheelPurePursuitMovement {

    static Telemetry telemetry;

    static BNO055IMU gyro;
    static Orientation angles;

    static double movementX = 0.0;
    static double movementY = 0.0;
    static double movementTurn = 0.0;

    private static DriveWheelPurePursuitDrivetrain pwr;
    private static DriveWheelOdometry odometry;

    public DriveWheelPurePursuitMovement(Telemetry tel, HardwareMap hardwareMap) {
        telemetry = tel;

        pwr = new DriveWheelPurePursuitDrivetrain(hardwareMap);
        odometry = new DriveWheelOdometry(hardwareMap, tel);

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

    public void init(HardwareMap hardwareMap, Telemetry tel) {
        pwr = new DriveWheelPurePursuitDrivetrain(hardwareMap);
        pwr.init();
        odometry = new DriveWheelOdometry(hardwareMap, tel);
        odometry.init();
        movementX = 0.0;
        movementY = 0.0;
        movementTurn = 0.0;
    }

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {

        CurvePoint followMe = getFollowPointPath(allPoints, new Point(odometry.xLocation,odometry.yLocation), allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

        telemetry.addData("FollowMePoint", "(%f,%f)", followMe.x, followMe.y);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {


        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i=0; i<pathPoints.size() - 1; i++) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint(), telemetry);

            telemetry.addData("Intersections", intersections);
            double closestAngle = 10000000;

            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - odometry.yLocation, thisIntersection.x - odometry.xLocation);
                double deltaAngle = Math.abs(PurePursuitMath.AngleWrap(angle - odometry.getRawHeading()));

                telemetry.addData("IntersectionAngle", Math.toDegrees(angle));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }


    /**
     * Basic run to a position. Used in followCurve.
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x*odometry.COUNTS_PER_INCH - odometry.xLocation, y*odometry.COUNTS_PER_INCH - odometry.yLocation);

        telemetry.addData("DistanceToTarget", distanceToTarget / odometry.COUNTS_PER_INCH);

        double absoluteAngleToTarget = Math.atan2(y*odometry.COUNTS_PER_INCH - odometry.yLocation, x*odometry.COUNTS_PER_INCH - odometry.xLocation);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (odometry.getRawHeading()));

        telemetry.addData("RelativeAngleToPoint", Math.toDegrees(relativeAngleToPoint));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        telemetry.addData("x,y to point", "%f,%f", relativeXToPoint/odometry.COUNTS_PER_INCH, relativeYToPoint/odometry.COUNTS_PER_INCH);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementX = movementXPower * movementSpeed;
        movementY = movementYPower * movementSpeed;

        pwr.ApplyPower();
        odometry.updateLocation();

        double relativeTurnAngle = -(relativeAngleToPoint + preferredAngle);//Maybe needs -Math.toRadians(180)
        movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1,1) * turnSpeed;

        if (distanceToTarget < 4*odometry.COUNTS_PER_INCH) {// < 4 inches
            movementTurn = 0;
        }
    }
}
