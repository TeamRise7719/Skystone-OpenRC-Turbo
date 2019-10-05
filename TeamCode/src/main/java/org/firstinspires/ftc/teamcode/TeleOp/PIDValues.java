package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by seancardosi on 10/05/19.
 */
@TeleOp(name = "PIDValues", group = "Testing")//This program should output the PID values based off of the selected motor in the configuration
public class PIDValues extends OpMode {

    DcMotor lf, lr, rf, rr;

    public void init() {


        lr = hardwareMap.dcMotor.get("leftB");
        lf = hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rr = hardwareMap.dcMotor.get("rightB");
        rf = hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void loop(){

        DcMotorControllerEx motorControllerExLB = (DcMotorControllerEx)lr.getController();
        DcMotorControllerEx motorControllerExLF = (DcMotorControllerEx)lf.getController();
        DcMotorControllerEx motorControllerExRB = (DcMotorControllerEx)rr.getController();
        DcMotorControllerEx motorControllerExRF = (DcMotorControllerEx)rf.getController();

        int motorIndexLB = ((DcMotorEx)lr).getPortNumber();
        PIDCoefficients pidOrigLB = motorControllerExLB.getPIDCoefficients(motorIndexLB, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorIndexLF = ((DcMotorEx)lf).getPortNumber();
        PIDCoefficients pidOrigLF= motorControllerExLF.getPIDCoefficients(motorIndexLF, DcMotor.RunMode.RUN_USING_ENCODER);

        int motorIndexRB = ((DcMotorEx)rr).getPortNumber();
        PIDCoefficients pidOrigRB = motorControllerExRB.getPIDCoefficients(motorIndexRB, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorIndexRF = ((DcMotorEx)rf).getPortNumber();
        PIDCoefficients pidOrigRF= motorControllerExRF.getPIDCoefficients(motorIndexRF, DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("P,I,D Left Back", "%.04f, %.04f, %.0f",
                pidOrigLB.p, pidOrigLB.i, pidOrigLB.d);
        telemetry.addData("P,I,D Left Front", "%.04f, %.04f, %.0f",
                pidOrigLF.p, pidOrigLF.i, pidOrigLF.d);
        telemetry.addData("P,I,D Right Back", "%.04f, %.04f, %.0f",
                pidOrigRB.p, pidOrigRB.i, pidOrigRB.d);
        telemetry.addData("P,I,D Right Front", "%.04f, %.04f, %.0f",
                pidOrigRF.p, pidOrigRF.i, pidOrigRF.d);

        telemetry.update();
    }

}
