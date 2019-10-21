package org.firstinspires.ftc.teamcode.PositionIntegration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PositionIntegration extends OpMode {

    private BNO055IMU gyro;
    private Orientation angle;
    private ElapsedTime elapsedTime;
    private double positionX = 0.0;
    private double positionY = 0.0;
    private double displacementX = 0.0;
    private double displacementY = 0.0;

    @Override
    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(param);
        angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        elapsedTime.reset();
    }

    @Override
    public void loop() {

        /*
         * Calculate displacement on the x and y axis using d = v_i * t + 1/2 * a * t^2.
         */
        displacementX = ((gyro.getVelocity().xVeloc) * (elapsedTime.seconds())) * ((0.5) * (gyro.getAcceleration().xAccel) * (Math.pow(elapsedTime.seconds(),2)));
        displacementY = ((gyro.getVelocity().yVeloc) * (elapsedTime.seconds())) * ((0.5) * (gyro.getAcceleration().yAccel) * (Math.pow(elapsedTime.seconds(),2)));

        elapsedTime.reset();//Resets the timer

        /*
         * Calculate the x and y positions.
         */
        positionX += displacementX;
        positionY += displacementY;

        telemetry.addData("Position(x,y)","(%d,%d)", positionX, positionY);

    }
}
