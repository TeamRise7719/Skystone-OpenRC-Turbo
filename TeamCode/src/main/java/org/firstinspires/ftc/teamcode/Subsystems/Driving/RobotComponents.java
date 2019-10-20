package org.firstinspires.ftc.teamcode.Subsystems.Driving;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Created by Sean Cardosi on 10/15/2019.
 */
public class RobotComponents {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DcMotor intake;
    private final DcMotor liftL;
    private final DcMotor liftR;
    private int liftMax = 1000;
    private int liftMin = 50;




    public RobotComponents(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftL = hardwareMap.dcMotor.get("liftL");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setDirection(DcMotor.Direction.REVERSE);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setDirection(DcMotor.Direction.FORWARD);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void init() {

        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //----------------------------------------------=+(Intake)+=----------------------------------------------\\
    public void intakeStone() {
        intake.setPower(1.0);
    }
    public void ejectStone() {
        intake.setPower(-1.0);
    }
    public void stopStone() {
        intake.setPower(0.0);
    }
    //----------------------------------------------=+(Intake)+=----------------------------------------------\\



    //TODO: Adjust lift encoder values to properly stop to avoid breaking the lift.
    //----------------------------------------------=+(Lift)+=----------------------------------------------\\
    public void liftControlUp(Gamepad gamepad) {//For driver control

        if (gamepad.dpad_up && liftL.getCurrentPosition() < liftMax) {//If liftL is below its max height, raise
            liftL.setPower(1.0);
        } else if (liftL.getCurrentPosition() >= liftMax) {//If liftL is above its max height, stop
            liftL.setPower(0.0);
        } else {
            liftStop();
        }
        if (gamepad.dpad_up && liftR.getCurrentPosition() < liftMax) {//If liftR is below its max height, raise
            liftR.setPower(1.0);
        } else if (liftR.getCurrentPosition() >= liftMax) {//If liftR is above its max height, stop
            liftR.setPower(0.0);
        } else {
            liftStop();
        }
    }

    public void liftControlDown(Gamepad gamepad) {//For driver control

        if (gamepad.dpad_down && liftL.getCurrentPosition() > liftMin) {//If liftL is above its min height, lower
            liftL.setPower(-1.0);
        } else if (liftL.getCurrentPosition() <= liftMin) {//If liftL is below its min height, stop
            liftL.setPower(0.0);
        } else {
            liftStop();
        }

        if (gamepad.dpad_down && liftR.getCurrentPosition() > liftMin) {//If liftR is above its min height, lower
            liftR.setPower(-1.0);
        } else if (liftR.getCurrentPosition() <= liftMin) {//If liftR is below its min height, stop
            liftR.setPower(0.0);
        } else {
            liftStop();
        }
    }

    //TODO: Convert the position to inches.
    public void liftPosition(int height) {//For autonomous

        liftL.setTargetPosition(height);
        liftR.setTargetPosition(height);

        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void  liftStop() {
        liftL.setPower(0.0);
        liftR.setPower(0.0);
    }
    //----------------------------------------------=+(Lift)+=----------------------------------------------\\


}
