package org.firstinspires.ftc.teamcode.Subsystems.Mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    DcMotor bl, br, fl, fr;

    public void init(HardwareMap hwMap){
        bl = hwMap.get(DcMotor.class, "bl");
        br = hwMap.get(DcMotor.class, "br");
        fl = hwMap.get(DcMotor.class, "fl");
        fr = hwMap.get(DcMotor.class, "fr");


        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //

    }

    public void drive(double forward, double turn, double rotate){
        double flPower = forward + turn + rotate;
        double frPower = forward - turn - rotate;
        double brPower = forward + turn - rotate;
        double blPower = forward - turn + rotate;
 

        double maxPower = 1.0;
        double maxSpeed = 0.7;

        maxPower = Math.max(maxPower, Math.abs(flPower));
        maxPower = Math.max(maxPower, Math.abs(frPower));
        maxPower = Math.max(maxPower, Math.abs(brPower));
        maxPower = Math.max(maxPower, Math.abs(blPower));

        fl.setPower(maxSpeed * (flPower / maxPower));
        fr.setPower(maxSpeed * (frPower / maxPower));
        bl.setPower(maxSpeed * (blPower / maxPower));
        br.setPower(maxSpeed * (brPower / maxPower));
    }

    public void switchZeroModeBehavior(DcMotor.ZeroPowerBehavior zeroPower){
        br.setZeroPowerBehavior(zeroPower);
        bl.setZeroPowerBehavior(zeroPower);
        fl.setZeroPowerBehavior(zeroPower);
        fr.setZeroPowerBehavior(zeroPower);
    }
}
