package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PurePursuit.Math.Point;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.PurePursuit.PurePursuitMath.lineCircleIntersection;

public class PurePursuitMovement {

    static Telemetry telemetry;

    static DcMotor leftBack;
    static DcMotor leftFront;
    static DcMotor rightBack;
    static DcMotor rightFront;

    static BNO055IMU gyro;
    static Orientation angles;

    double movementX = 0;

    public PurePursuitMovement(Telemetry tel, HardwareMap hardwareMap) {
        telemetry = tel;

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        leftBack = hardwareMap.dcMotor.get("leftB");
        leftFront = hardwareMap.dcMotor.get("leftF");
        rightBack = hardwareMap.dcMotor.get("rightB");
        rightFront = hardwareMap.dcMotor.get("rightF");
    }

    public static void init() {

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {


        CurvePoint followMe = getFollowPointPath(allPoints, new Point(CurrentXPosition/*This should be the robots current x position*/), allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

        telemetry.addData("FollowMePoint", "(%f,%f)", followMe.x, followMe.y);

    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double xPos, double yPos, double followRadius) {

        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i=0; i<pathPoints.size() - 1; i++) {

            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 10000000;

            for (Point thisIntersection : intersections) {

                double angle = Math.atan2(thisIntersection.y - CurrentYPosition, thisIntersection.x - CurrentXPosition);
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
     * Basic run to a position. Better to use followCurve.
     * @param x
     * @param y
     * @param movementSpeed
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(x - CurrentXPosition, y - CurrentYPosition);

        double absoluteAngleToTarget = Math.atan2(y - CurrentYPosition, x - CurrentXPosition);//WorldPosition is the robots starting position and should be updated to the robots current position throughout OpMode

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (angles.firstAngle - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint * distanceToTarget);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementX = movementXPower * movementSpeed;//Movement X and Y are the power to apply to the motors
        movementY = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        movementTurn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1,1) * turnSpeed;//movement Turn is power to turn angles

        if (distanceToTarget < 10) {// < 10cm
            movementTurn = 0;
        }
    }
}
