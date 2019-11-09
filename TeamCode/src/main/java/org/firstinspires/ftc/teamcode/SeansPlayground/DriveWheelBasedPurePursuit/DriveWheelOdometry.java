package org.firstinspires.ftc.teamcode.SeansPlayground.DriveWheelBasedPurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * Created by Sean Cardosi on 11/6/2019
 * Class used for finding the robots current angle and location on the field.
 */
public class DriveWheelOdometry {

    private final DcMotor lf, lr, rf, rr;

    public BNO055IMU gyro;
    public Orientation angles;

    Telemetry telemetry;

    double xLocation = 0.0;
    double yLocation = 0.0;
    double distance = 0.0;
    double changeRight = 0.0;
    double changeLeft = 0.0;
    double previousRightValue = 0.0;
    double previousLeftValue = 0.0;
    private double COUNTS_PER_REVOLUTION = 360;
    private double WHEEL_DIAMETER_MM = 38;
    private double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_MM * 10;
    private double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_CM;
    private double COUNTS_PER_CM = COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;


    public DriveWheelOdometry(HardwareMap hardwareMap, Telemetry tel) {

        //GYRO IS IN RADIANS FOR PURE PURSUIT
        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        lr = hardwareMap.dcMotor.get("leftB");
        lf = hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rr = hardwareMap.dcMotor.get("rightB");
        rf = hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = tel;
    }
    public void init () {

        xLocation = 0.0;
        yLocation = 0.0;
        distance = 0.0;
        changeRight = 0.0;
        changeLeft = 0.0;
        previousRightValue = 0.0;
        previousLeftValue = 0.0;
    }

    /*
     * Updates the previous encoder values..
     */
    public void previousValues() {

        previousRightValue = (rr.getCurrentPosition() + rf.getCurrentPosition()) / 2.0;
        previousLeftValue = (lr.getCurrentPosition() + lf.getCurrentPosition()) / 2.0;
    }

    /*
     * Finds the robots (x,y) location using the previous encoder values and the robot heading.
     */
    public void updateLocation() {

        changeRight = ((rr.getCurrentPosition() + rf.getCurrentPosition()) / 2.0) - previousRightValue;
        changeLeft = ((lr.getCurrentPosition() + lf.getCurrentPosition()) / 2.0) - previousLeftValue;

        distance = (changeRight + changeLeft) / 2.0;
        xLocation += (distance * Math.cos(angles.firstAngle));// * COUNTS_PER_CM;
        yLocation += (distance * Math.sin(angles.firstAngle));// * COUNTS_PER_CM;

//        telemetry.addData("Location in CM: ", "(%d,%d)", xLocation, yLocation);
        telemetry.addData("xLocation", xLocation);
        telemetry.addData("yLocation", yLocation);

        previousValues();
    }
}
