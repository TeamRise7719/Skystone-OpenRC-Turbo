package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.movementTurn;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.movementX;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.movementY;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement.telemetry;

/**
 * Created by Sean Cardosi on 11/6/2019
 * DriveWheelPurePursuitDrivetrain is the drivetrain class to only be used with Pure Pursuit.
 */
public class DriveWheelPurePursuitDrivetrain {

    public static DcMotor lf, lr, rf, rr;
    private final HardwareMap hardwareMap;
    double lfPower = 0.0;
    double lrPower = 0.0;
    double rfPower = 0.0;
    double rrPower = 0.0;


    public DriveWheelPurePursuitDrivetrain(final HardwareMap _hardwareMap) {
        hardwareMap = _hardwareMap;

        //configuring the components
        lr = hardwareMap.dcMotor.get("leftB");
        lf = hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rr = hardwareMap.dcMotor.get("rightB");
        rf = hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        lfPower = 0.0;
        lrPower = 0.0;
        rfPower = 0.0;
        rrPower = 0.0;
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//This might need to be done in Odometry classes
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ApplyPower() {

        double x = movementX;
        double y = movementY;
        double r = movementTurn;

        lfPower = x + y + r;//Maybe *1.5 movementY - movementTurn + movementX
        lrPower = x - y + r;//movementY - movementTurn - movementX
        rfPower = x - y - r;//-movementY - movementTurn + movementX
        rrPower = x + y - r;//-movementY - movementTurn - movementX

        //find the maximum of the powers
        double lfmaxRawPower = Math.abs(lfPower);
        double rfmaxRawPower = Math.abs(rfPower);
        double lrmaxRawPower = Math.abs(lrPower);
        double rrmaxRawPower = Math.abs(rrPower);
        if(Math.abs(lfPower) > lfmaxRawPower){ lfmaxRawPower = Math.abs(lrPower);}
        if(Math.abs(lrPower) > lrmaxRawPower){ lrmaxRawPower = Math.abs(lrPower);}
        if(Math.abs(rrPower) > rrmaxRawPower){ rrmaxRawPower = Math.abs(rrPower);}
        if(Math.abs(rfPower) > rfmaxRawPower){ rfmaxRawPower = Math.abs(rfPower);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double lfscaleDownAmount = 1.0;
        double lrscaleDownAmount = 1.0;
        double rrscaleDownAmount = 1.0;
        double rfscaleDownAmount = 1.0;
        if(lfmaxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            lfscaleDownAmount = 1.0/lfmaxRawPower;
        }
        if(lrmaxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            lrscaleDownAmount = 1.0/lrmaxRawPower;
        }
        if(rrmaxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            rrscaleDownAmount = 1.0/rrmaxRawPower;
        }
        if(rfmaxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            rfscaleDownAmount = 1.0/rfmaxRawPower;
        }

        lfPower *= lfscaleDownAmount;
        lrPower *= lrscaleDownAmount;
        rrPower *= rrscaleDownAmount;
        rfPower *= rfscaleDownAmount;

        lf.setPower(lfPower);//Power is negative because location was updated in the wrong direction (negative)
        lr.setPower(lrPower);
        rf.setPower(rfPower);
        rr.setPower(rrPower);

        telemetry.addData("LeftFront ", lfPower);
        telemetry.addData("LeftBack ", lrPower);
        telemetry.addData("RightFront ", rfPower);
        telemetry.addData("RightBack ", rrPower);
    }
}
