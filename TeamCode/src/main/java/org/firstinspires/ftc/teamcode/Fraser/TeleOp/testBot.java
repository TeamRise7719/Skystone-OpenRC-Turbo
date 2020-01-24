package org.firstinspires.ftc.teamcode.Fraser.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="7th Period Code")
public class testBot extends LinearOpMode {

    DcMotor driveL, driveR, arm;
    Servo clawL, clawR;

    @Override
    public void runOpMode() throws InterruptedException {

        driveL = hardwareMap.dcMotor.get("dL");
        driveR = hardwareMap.dcMotor.get("dR");
        arm = hardwareMap.dcMotor.get("arm");

        driveL.setDirection(DcMotor.Direction.FORWARD);
        driveR.setDirection(DcMotor.Direction.REVERSE);

        driveL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawL = hardwareMap.servo.get("R");
        clawR = hardwareMap.servo.get("L");

        clawL.setDirection(Servo.Direction.FORWARD);
        clawR.setDirection(Servo.Direction.REVERSE);

        clawL.setPosition(0.1);
        clawR.setPosition(0.1);
        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.a){
                clawL.setPosition(0.5);
                clawR.setPosition(0.5);
            } else if (gamepad1.y){
                clawR.setPosition(0);
                clawL.setPosition(0);
            }

            driveL.setPower(gamepad1.left_stick_y);
            driveR.setPower(gamepad1.right_stick_y);

            arm.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        }
    }
}
