package org.firstinspires.ftc.teamcode.Fraser.RoadRunner.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.Fraser.RoadRunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Fraser.RoadRunner.util.BNO055IMUUtil;

import java.util.Arrays;
import java.util.List;

/**
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |              |
 *    |              |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 *
 * Created by Sean Cardosi on 2019-12-21.
 */
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 360;
    public static double WHEEL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels

    private DcMotor leftOdometer, rightOdometer;
    private BNO055IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0) // right
        ));

        leftOdometer = hardwareMap.dcMotor.get("leftEncoder");
        rightOdometer = hardwareMap.dcMotor.get("rightEncoder");

        imu = hardwareMap.get(BNO055IMU.class, "imuINT");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);//Rempaing the axis for a vertical hub
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftOdometer.getCurrentPosition()),
                encoderTicksToInches(rightOdometer.getCurrentPosition())
        );
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
