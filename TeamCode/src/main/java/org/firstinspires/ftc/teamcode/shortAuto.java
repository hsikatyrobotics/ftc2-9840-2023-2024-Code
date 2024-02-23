package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardwareMap;


@Autonomous
public class shortAuto extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor flm = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor blm = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frm = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor brm = hardwareMap.dcMotor.get("backRightMotor");
        Servo cs1 = hardwareMap.servo.get("cs1");
        Servo cs2 = hardwareMap.servo.get("cs2");


        cs2.setPosition(0.8);
        cs1.setPosition(0);

        waitForStart();

        flm.setPower(0.5);
        frm.setPower(0.5);
        blm.setPower(0.5);
        brm.setPower(0.5);

        sleep(1750);

        flm.setPower(0);
        frm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);

        sleep(1000);

        cs2.setPosition(0.6);
        cs1.setPosition(0.55);

        sleep(1000);

        flm.setPower(0.2);
        frm.setPower(0.2);
        blm.setPower(0.2);
        brm.setPower(0.25);

        sleep(1000);




    }
}
