package org.firstinspires.ftc.teamcode.Fraser.Autonomous.Tuning;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive.SeansEncLibrary;

import java.util.ArrayList;

/**
 * Created by Sean Cardosi on 2020-01-11.
 */
@Autonomous(name = "Track Width Tuner", group = "Tuning")
public class TrackWidthTuner extends LinearOpMode {

    private SeansEncLibrary enc;
    private BNO055IMU gyro;
    private double trackWidth;
    private double offset = 0;


    private final int Trials                    = 5;
    private final int timeBetweenTests          = 5000;
    private final double MeasuredTrackWidth     = 13.5;
    private final double targetAngle            = 180;


    @Override
    public void runOpMode() {
        enc = new SeansEncLibrary(hardwareMap,telemetry,this);
        enc.init();

        gyro = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(param);
        gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addLine("Tuner is Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Tuner Started...");
        telemetry.update();

        sleep(1000);

        telemetry.addLine("Tuning...");
        telemetry.update();

        ArrayList<Double> trackWidths = new ArrayList<>();

        /*
        I'm tired so this makes sense:
        Get percent of angle we were supposed to turn.
        Trackwidth *= %error
        */

        for (int i=1;i<=Trials;i++) {

            double calculated;

            resetHeading();

            telemetry.addData("Test Number", i);
            telemetry.update();

            enc.arcTurn(targetAngle);

            double percentErr = getAdjustedHeading() / targetAngle;

            calculated = MeasuredTrackWidth * percentErr;

            trackWidths.add(calculated);

            sleep(timeBetweenTests);
        }

        for (Double d : trackWidths) {
            trackWidth += d;
        }
        trackWidth /= trackWidths.size();

        while (opModeIsActive()) {
            telemetry.addData("Tuned Track Width", trackWidth);
            telemetry.update();
        }
    }

    private double getAdjustedHeading() {
        return gyro.getAngularOrientation().firstAngle - offset;
    }
    private void resetHeading() {
        offset = gyro.getAngularOrientation().firstAngle;
    }
}
