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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Sean Cardosi on 10/21/2019.
 * Just like before, this isn't going to work.
 */
@TeleOp(name = "PreMadeIMUIntegration", group = "Integration")
public class PositionBNO055Function extends OpMode {

    private BNO055IMU gyro;
    private Orientation angle;
    private Position position;
    private Velocity velocity;


    public void init() {

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        gyro.initialize(param);
        angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyro.startAccelerationIntegration(position,velocity,15);
    }
    public void loop() {

        telemetry.addData("GyroPosition(x,y)","(%f,%f)", gyro.getPosition().x, gyro.getPosition().y);

        telemetry.addData("NavigationPosition(x,y)","(%f,%f)", position.x, position.y);
    }
}
