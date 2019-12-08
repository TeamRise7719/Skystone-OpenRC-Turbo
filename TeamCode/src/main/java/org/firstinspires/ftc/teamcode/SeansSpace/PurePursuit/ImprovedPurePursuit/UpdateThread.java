package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.ImprovedPurePursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelOdometry;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitDrivetrain;
import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.ImprovedPurePursuit.DrivetrainMovement;

/**
 * Created by Sean Cardosi on 2019-12-05.
 */
public class UpdateThread implements Runnable {

    private boolean isRunning = true;
    DriveWheelOdometry odometry;
    DrivetrainMovement drivetrain;


    /**
     * Initialization method
     * @param hardwareMap
     * @param telemetry
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        odometry = new DriveWheelOdometry(hardwareMap, telemetry);
        odometry.init();
        drivetrain = new DrivetrainMovement(hardwareMap);
        drivetrain.init();
    }

    @Override
    public void run() {
        while (isRunning) {
            odometry.updateLocation();
            drivetrain.ApplyPower();
        }
    }

    /**
     * Stop method
     */
    public void stop() {
        isRunning = false;
    }
}
