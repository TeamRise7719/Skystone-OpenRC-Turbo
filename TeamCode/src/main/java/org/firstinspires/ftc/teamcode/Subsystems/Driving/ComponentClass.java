package org.firstinspires.ftc.teamcode.Subsystems.Driving;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Created by Sean Cardosi on 10/15/2019.
 */
public class ComponentClass {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DcMotor intake;


    public ComponentClass(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;


        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intakeStone(double power) {
        intake.setPower(power);
    }
    public void ejectStone(double power) {
        intake.setPower(power);
    }
    public void stopStone() {
        intake.setPower(0.0);
    }
}
