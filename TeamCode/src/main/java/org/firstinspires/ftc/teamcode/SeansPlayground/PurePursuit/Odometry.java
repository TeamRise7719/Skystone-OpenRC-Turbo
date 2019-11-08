package org.firstinspires.ftc.teamcode.SeansPlayground.PurePursuit;

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
 * Created by Sean Cardosi on 10/17/2019.
 * Class used for finding the robots current angle and location on the field.
 */
public class Odometry {

    DcMotor leftOdometer;
    DcMotor rightOdometer;

    public BNO055IMU gyro;
    public Orientation angles;

    Telemetry telemetry;

    double xLocation = 0.0;
    double yLocation = 0.0;
    double distance;
    double changeRight = 0.0;
    double changeLeft = 0.0;
    double previousRightValue = 0.0;
    double previousLeftValue = 0.0;
    private double COUNTS_PER_REVOLUTION = 360;
    private double WHEEL_DIAMETER_MM = 38;
    private double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_MM * 10;
    private double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_CM;
    private double COUNTS_PER_CM = COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;


    public Odometry(HardwareMap hardwareMap, Telemetry tel) {
        //GYRO IS IN RADIANS FOR PURE PURSUIT

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        leftOdometer.setDirection(DcMotor.Direction.REVERSE);
        leftOdometer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightOdometer.setDirection(DcMotor.Direction.FORWARD);
        rightOdometer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOdometer = hardwareMap.dcMotor.get("encL");
        rightOdometer = hardwareMap.dcMotor.get("encR");

        telemetry = tel;
    }

    /*
     * Updates the previous encoder values..
     */
    public void previousValues() {

        previousRightValue = rightOdometer.getCurrentPosition();
        previousLeftValue = leftOdometer.getCurrentPosition();
    }

    /*
     * Finds the robots (x,y) location using the previous encoder values and the robot heading.
     */
    public void updateLocation() {

        changeRight = rightOdometer.getCurrentPosition() - previousRightValue;
        changeLeft = leftOdometer.getCurrentPosition() - previousLeftValue;

        distance = (changeRight + changeLeft) / 2;
        xLocation += (distance * Math.cos(angles.firstAngle)) * COUNTS_PER_CM;
        yLocation += (distance * Math.sin(angles.firstAngle)) * COUNTS_PER_CM;

        telemetry.addData("Location in CM: ", "%d:%d", xLocation, yLocation);

        previousValues();
    }
}