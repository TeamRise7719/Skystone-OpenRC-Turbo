package org.firstinspires.ftc.teamcode.Subsystems.Driving;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    private int liftMax = 2000;
    private int liftMin = 50;
//    private final Servo rot;
//    private final Servo grab;
//    private final Servo arm;






    public RobotComponents(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftL = hardwareMap.dcMotor.get("liftL");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setDirection(DcMotor.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rot = hardwareMap.servo.get("rot");
//        grab = hardwareMap.servo.get("grab");
//        arm = hardwareMap.servo.get("arm");
    }
    public void init() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //----------------------------------------------=+(Intake)+=----------------------------------------------\\
    public void intakeStone() {
        intake.setPower(1);
    }
    public void ejectStone() {
        intake.setPower(-1);
    }
    public void stopStone() {
        intake.setPower(0.0);
    }
    //----------------------------------------------=+(Intake)+=----------------------------------------------\\



    //TODO: Adjust lift encoder values to properly stop to avoid breaking the lift.
    //----------------------------------------------=+(Lift)+=----------------------------------------------\\

    /**
     * Uses a gamepad input to assign the upward movement of the lift to
     * @param gamepad The gamepad to be sued to control the upward movement of the lift
     */
    public void liftControlUp(Gamepad gamepad) {//For driver control
        liftL.setPower(gamepad.left_stick_y);
        liftR.setPower(gamepad.left_stick_y);
    }

    /**
     * Uses a gamepad number to assign the downward movement of the lift to
     * @param gamepad The gamepad to be used to control the downward movement of the lift
     */


    /**
     * Makes the lift run to a certain height.
     * @param height The height for the lift to raise to. Currently in encoder ticks.
     *               Needs to be converted into inches.
     * @param velocity The speed and direction for the lift to run at. -1 goes down and 1 goes up
     */
    public void liftPosition(int height, double velocity) {//TODO: Convert the position to inches.

        liftL.setTargetPosition(height);
        liftR.setTargetPosition(height);

        liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftL.setPower(velocity);
        liftR.setPower(velocity);
    }

    /**
     * Stops the lifts movement
     */
    public void  liftStop() {
        liftL.setPower(0.0);
        liftR.setPower(0.0);
    }
    //----------------------------------------------=+(Lift)+=----------------------------------------------\\


//    //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
//    public void returnPosition() {
//
//    }
//    public void outPosition() {
//
//    }
//    //----------------------------------------------=+(Grabber)+=----------------------------------------------\\

}
