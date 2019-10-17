package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * Created by Sean Cardosi on 10/17/2019.
 */
public class Odometry {

    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;

    public BNO055IMU gyro;
    public Orientation angles;

    Telemetry telemetry;
    LinearOpMode linearOpMode;

    private static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    private static final double     EXTERNAL_GEAR_RATIO    = (25/32);     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private double xLocation = 0;
    private double yLocation = 0;
    private double xLocationInch = 0;
    private double yLocationInch = 0;
    private double distance;
    private double distanceInch;
    private double changeRight = 0;
    private double changeLeft = 0;
    private double previousRightValue = 0;
    private double previousLeftValue = 0;



    public Odometry(HardwareMap hardwareMap, Telemetry tel, LinearOpMode opMode) {
        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");

        leftBack = hardwareMap.dcMotor.get("leftB");
        leftFront = hardwareMap.dcMotor.get("leftF");
        rightBack = hardwareMap.dcMotor.get("rightB");
        rightFront = hardwareMap.dcMotor.get("rightF");

        telemetry = tel;
        linearOpMode = opMode;
    }

    public void init(){

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

    public void stop_all_motors(){

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /*
     * Updates the previous encoder values. Call after each point is run to.
     */
    public void previousValues() {

        previousRightValue = (rightBack.getCurrentPosition()+rightFront.getCurrentPosition())/2;
        previousLeftValue = (leftBack.getCurrentPosition()+leftFront.getCurrentPosition())/2;
    }

    /*
     * Finds the robots (x,y) location using the previous encoder values and the robot heading.
     */
    public void Location() {

        changeRight = ((rightBack.getCurrentPosition()+rightFront.getCurrentPosition())/2) - previousRightValue;
        changeLeft = ((leftBack.getCurrentPosition()+leftFront.getCurrentPosition())/2) - previousLeftValue;

        distance = (changeRight+changeLeft)/2;
        distanceInch = (distance*COUNTS_PER_INCH);

        xLocation += distance*Math.cos(angles.firstAngle);
        yLocation += distance*Math.sin(angles.firstAngle);
        xLocationInch += distanceInch*Math.cos(angles.firstAngle);
        yLocationInch +=distanceInch*Math.sin(angles.firstAngle);

        telemetry.addData("Location:", "%f:%f",xLocation, yLocation);
        telemetry.addData("Location In Inches: ", "%d:%d", xLocationInch, yLocationInch);
    }
}
