package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.0)
            .forwardZeroPowerAcceleration(-47.89428370842306)
            .lateralZeroPowerAcceleration(-59.453233028422474 )
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0.0003, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(-1, 0, 0.0003, 0.03))
            ;



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")
            .leftRearMotorName("back_left_motor")
            .leftFrontMotorName("front_left_motor")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotor.Direction.REVERSE)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(-65.21317946066068)
            .yVelocity(57.906141386257374);





    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();


    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-90.0)
            .strafePodX(-85.0)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .customEncoderResolution(19.8943)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);





}