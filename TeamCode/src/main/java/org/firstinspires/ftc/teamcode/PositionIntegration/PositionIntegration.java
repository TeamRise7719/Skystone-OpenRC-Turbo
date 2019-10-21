package org.firstinspires.ftc.teamcode.PositionIntegration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Sean Cardosi on 10/21/2019.
 * I don't think you can loop fast enough for this to work.
 */
@TeleOp(name = "SelfPositionIntegration", group = "Integration")
public class PositionIntegration extends OpMode {

    private BNO055IMU gyro;
    private Orientation angle;
    private ElapsedTime elapsedTime;
    private double positionX = 0.0;
    private double positionY = 0.0;
    private double displacementX;
    private double displacementY;
    private double sX;
    private double sY;
    private double pX = 0.0;
    private double pY = 0.0;
    private double prevVelx = 0.0;
    private double prevVely = 0.0;
    private double displacementX1;
    private double displacementY1;
    private double prevVelx1 = 0.0;
    private double prevVely1 = 0.0;
    private double positionX1 = 0.0;
    private double positionY1 = 0.0;

    @Override
    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(param);
        angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        elapsedTime.reset();
    }

    @Override
    public void loop() {

        /*
         * Calculate displacement on the x and y axis using s = v_i * t + 1/2 * a * t^2.
         * Could also probably use s = v*∆t
         */
        displacementX = ((gyro.getVelocity().xVeloc) * (elapsedTime.seconds())) * ((0.5) * (gyro.getAcceleration().xAccel) * (Math.pow(elapsedTime.seconds(),2)));
        displacementY = ((gyro.getVelocity().yVeloc) * (elapsedTime.seconds())) * ((0.5) * (gyro.getAcceleration().yAccel) * (Math.pow(elapsedTime.seconds(),2)));

        sX = (gyro.getVelocity().xVeloc - prevVelx) * elapsedTime.seconds();
        sY = (gyro.getVelocity().yVeloc - prevVely) * elapsedTime.seconds();

        displacementX1 = ((gyro.getVelocity().xVeloc - prevVelx1) * (elapsedTime.seconds())) * ((0.5) * (gyro.getAcceleration().xAccel) * (Math.pow(elapsedTime.seconds(),2)));
        displacementY1 = ((gyro.getVelocity().yVeloc - prevVely1) * (elapsedTime.seconds())) * ((0.5) * (gyro.getAcceleration().yAccel) * (Math.pow(elapsedTime.seconds(),2)));

        elapsedTime.reset();//Resets the timer

        /*
         * Calculate the x and y positions.
         */
        positionX += displacementX;
        positionY += displacementY;
        pX += sX;
        pY += sY;
        positionX1 += displacementX1;
        positionY1 += displacementY1;

        telemetry.addData("Position(x,y) = v∆t+1/2a∆t^2","(%d,%d)", positionX, positionY);
        telemetry.addData("Position(x,y) = ∆v∆t","(%d,%d)", pX, pY);
        telemetry.addData("Position(x,y) = ∆v∆t+1/2a∆t^2","(%d,%d)", positionX1, positionY1);

        prevVelx = gyro.getVelocity().xVeloc;
        prevVely = gyro.getVelocity().yVeloc;
        prevVelx1 = gyro.getVelocity().xVeloc;
        prevVely1 = gyro.getVelocity().yVeloc;
    }
}
