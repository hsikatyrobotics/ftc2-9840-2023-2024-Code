package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardwareMap;


@Autonomous
public class shortAuto extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor flm = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor blm = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frm = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor brm = hardwareMap.dcMotor.get("backRightMotor");

        flm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        flm.setPower(0.5);
        frm.setPower(0.5);
        blm.setPower(0.5);
        brm.setPower(0.5);

        sleep(7000);

        flm.setPower(0);
        frm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);

        sleep(100);

        flm.setPower(-0.2);
        frm.setPower(-0.2);
        blm.setPower(-0.2);
        brm.setPower(-0.2);

        sleep(100);




    }
}
