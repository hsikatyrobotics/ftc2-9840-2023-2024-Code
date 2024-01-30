package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class longBlueAuto extends LinearOpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo clawServo2 = hardwareMap.servo.get("clawServo2");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        clawServo.setPosition(0.85);
        clawServo2.setPosition(0.2);

        waitForStart();


        frontLeftMotor.setPower(-0.3);
        frontRightMotor.setPower(.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(-0.3);
        sleep(260);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(1000);
        frontLeftMotor.setPower(-0.8);
        frontRightMotor.setPower(-0.8);
        backLeftMotor.setPower(-0.8);
        backRightMotor.setPower(-0.8);
        sleep(2000);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        clawServo2.setPosition(0.9);
        clawServo.setPosition(0.6);








    }
}