package org.firstinspires.ftc.teamcode.Fraser.Subsystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Created by Sean Cardosi on 10/15/2019.
 */
public class FraserComponents {

    public final DcMotor intake;
    public final DcMotor liftL;
    public final DcMotor liftR;

    private Servo lgrab , rgrab, shoulderL ,shoulderR, claw;

    private double posit = 0.14;
    private int stackHeight = 0;
    private double HEIGHT_PER_BLOCK = 0;//TODO: Measure the height needed for the lift to move o place a block
    private boolean isLifting = false;

    public FraserComponents(final HardwareMap hardwareMap) {

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        liftL = hardwareMap.dcMotor.get("liftL");
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR = hardwareMap.dcMotor.get("liftR");
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setDirection(DcMotor.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lgrab = hardwareMap.servo.get("lgrab");
        rgrab = hardwareMap.servo.get("rgrab");
        rgrab.setDirection(Servo.Direction.REVERSE);

        shoulderL = hardwareMap.servo.get("shoulderL");
        shoulderR = hardwareMap.servo.get("shoulderR");
        shoulderR.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");
    }
    public void init() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //----------------------------------------------=+(Intake)+=----------------------------------------------\\
    public void intakeStone() {
        intake.setPower(-1);
    }
    public void ejectStone() {
        intake.setPower(1);
    }
    public void stopStone() {
        intake.setPower(0.0);
    }

    public void autoIntake(){
        intake.setPower(-0.8);
    }
    //----------------------------------------------=+(Intake)+=----------------------------------------------\\



    //----------------------------------------------=+(Lift)+=----------------------------------------------\\

    /**
     * Uses a gamepad input to assign the upward movement of the lift to
     * @param gamepad The gamepad to be used to control the upward movement of the lift
     */
    public void liftManual(Gamepad gamepad) {//For driver control
        liftL.setPower(gamepad.left_stick_y/2);
        liftR.setPower(gamepad.left_stick_y/2);
    }

    /**
     * Stops the lifts movement
     */
    public void  liftStop() {
        liftL.setPower(0.0);
        liftR.setPower(0.0);
    }



    //----------------------------------------------=+(Lift)+=----------------------------------------------\\



    //----------------------------------------------=+(Block Build)+=----------------------------------------------\\
    public void clawGrab(){
        claw.setPosition(0.1);
    }

    public void clawRelease(){
        claw.setPosition(0.5);
    }


    public void autoClaw(){
        claw.setPosition(0.3);
    }

//
//    public void wristLeft(Gamepad gamepad){
//        wrist.setPosition(gamepad.right_trigger/5);
//    }

//    public void shoulderOld(Gamepad gamepad){
//        double down = 0.13;
//        double up = 0.8;
//        if (gamepad.a){
//            posit = down;
//        } else if (gamepad.y){
//            posit = up;
//        }
//
//        shoulderL.setPosition(posit + gamepad.right_stick_y/10);
//        shoulderR.setPosition(posit + gamepad.right_stick_y/10);
//    }
//
    public void shoulderUp(){
        shoulderL.setPosition(0.25);
        shoulderR.setPosition(0.25);
    }
    public void shoulderDown(){
        shoulderL.setPosition(1);
        shoulderR.setPosition(1);
    }
    //----------------------------------------------=+(Block Build)+=----------------------------------------------\\



    //----------------------------------------------=+(Grabber)+=----------------------------------------------\\
    public void foundationRelease() {
        lgrab.setPosition(0.8); //.75
        rgrab.setPosition(0.8); //.75

    }
    public void foundationGrab() {
        lgrab.setPosition(0.05);
        rgrab.setPosition(0.05);

    }
    public void foundationInit(){
        lgrab.setPosition(0.05);
        rgrab.setPosition(0.05);
    }
    //----------------------------------------------=+(Grabber)+=----------------------------------------------\\






    //############################################## Driver Macros ##############################################






    //----------------------------------------------=+(Auto Stack)+=----------------------------------------------\\
    public void autoLeveler(Gamepad gamepad2) {
        if (gamepad2.x && !isLifting) {
            stackHeight++;
            isLifting = true;
            int height = (int)(stackHeight*HEIGHT_PER_BLOCK);
            liftL.setTargetPosition(height);
            liftR.setTargetPosition(height);
            liftL.setPower(1);
            liftR.setPower(1);
        } else if (gamepad2.x && isLifting) {
            isLifting = true;//This is just here to prevent the if statement from reaching else if you are still holding x.
        } else {
            liftManual(gamepad2);
            isLifting = false;
        }

    }
    //----------------------------------------------=+(Auto Stack)+=----------------------------------------------\\

}
