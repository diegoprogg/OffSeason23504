package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Mecanum.MecanumDriveFieldOriented;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    MecanumDriveFieldOriented drive;


    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, turn, rotate);

        if(gamepad1.right_bumper){
            drive.switchZeroModeBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (gamepad1.left_bumper) {
            drive.switchZeroModeBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


    }
}
