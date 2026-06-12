package org.firstinspires.ftc.teamcode.Subsystems.Mecanum;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MecanumDriveFieldOriented {
    DcMotor bl, br, fl, fr;
    GoBildaPinpointDriver odo;

    private static final double XOFFSET_MM = 90;
    private static final double YOFFSET_MM = 80;

    public void init(HardwareMap hwMap){
        bl = hwMap.get(DcMotor.class, "bl");
        br = hwMap.get(DcMotor.class, "br");
        fl = hwMap.get(DcMotor.class, "fl");
        fr = hwMap.get(DcMotor.class, "fr");


        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(XOFFSET_MM, YOFFSET_MM, DistanceUnit.MM);


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

    public void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - odo.getHeading(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }


    public void switchZeroModeBehavior(DcMotor.ZeroPowerBehavior zeroPower){
        br.setZeroPowerBehavior(zeroPower);
        bl.setZeroPowerBehavior(zeroPower);
        fl.setZeroPowerBehavior(zeroPower);
        fr.setZeroPowerBehavior(zeroPower);
    }

}
