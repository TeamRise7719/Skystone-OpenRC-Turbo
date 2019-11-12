package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.DriveWheelBasedPurePursuit.DriveWheelPurePursuitMovement;

import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.movementTurn;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.movementX;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.movementY;
import static org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.OdometerBasedPurePursuit.PurePursuitMovement.telemetry;



/**
 * Created by Sean Cardosi.
 * PurePursuitDrivetrain is the drivetrain class to only be used with Pure Pursuit.
 */
public class PurePursuitDrivetrain {

    public static DcMotor lf, lr, rf, rr;
    private final HardwareMap hardwareMap;
    double lfPower = 0.0;
    double lrPower = 0.0;
    double rfPower = 0.0;
    double rrPower = 0.0;

    public PurePursuitDrivetrain(final HardwareMap _hardwareMap) {
        hardwareMap = _hardwareMap;

        //configuring the components
        lr = hardwareMap.dcMotor.get("leftB");
        lf = hardwareMap.dcMotor.get("leftF");
        lr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rr = hardwareMap.dcMotor.get("rightB");
        rf = hardwareMap.dcMotor.get("rightF");
        rr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        lfPower = 0.0;
        lrPower = 0.0;
        rfPower = 0.0;
        rrPower = 0.0;
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//This night need to be done in Odometry classes
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ApplyPower() {

        lfPower = movementX - movementTurn - movementY;//Maybe *1.5 movementY - movementTurn + movementX
        lrPower = movementX - movementTurn + movementY;//movementY - movementTurn - movementX
        rfPower = movementX + movementTurn + movementY;//-movementY - movementTurn + movementX
        rrPower = movementX + movementTurn - movementY;//-movementY - movementTurn - movementX

        //find the maximum of the powers
        double maxRawPower = Math.abs(lfPower);
        if(Math.abs(lrPower) > maxRawPower){ maxRawPower = Math.abs(lrPower);}
        if(Math.abs(rrPower) > maxRawPower){ maxRawPower = Math.abs(rrPower);}
        if(Math.abs(rfPower) > maxRawPower){ maxRawPower = Math.abs(rfPower);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }

        lfPower *= scaleDownAmount;
        lrPower *= scaleDownAmount;
        rrPower *= scaleDownAmount;
        rfPower *= scaleDownAmount;

        lf.setPower(lfPower);//Power is negative because location was updated in the wrong direction (negative)
        lr.setPower(lrPower);
        rf.setPower(rfPower);
        rr.setPower(rrPower);

        telemetry.addData("MotorPowers\n",
                "   |lf------------rf|\n" +
                        "|%f|----------------|%f|\n" +
                        "   |                |\n" +
                        "   |                |\n" +
                        "   |                |\n" +
                        "|%f|lr------------rr|%f|\n" +
                        "   |----------------|\n",
                lfPower, rfPower, lrPower, rrPower);//Sorry Evan.
        telemetry.update();
    }
}
