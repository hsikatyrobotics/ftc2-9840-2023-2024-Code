package org.firstinspires.ftc.teamcode.fTeleop;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.tensorflow.lite.TensorFlowLite;


@Config
@TeleOp
public class FteleOp extends LinearOpMode {
    private PIDController controller;


    public int z = 0;


    boolean var = true;

    boolean var2 = true;

    boolean othervar = true;

    boolean othervar2 = true;



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rs = hardwareMap.dcMotor.get("rightSlide");
        DcMotor ls = hardwareMap.dcMotor.get("leftSlide");
        DcMotor claw = hardwareMap.dcMotor.get("clawArmMotor");


        Servo cs1 = hardwareMap.servo.get("cs1");
        Servo cs2 = hardwareMap.servo.get("cs2");
        Servo plane = hardwareMap.servo.get("planeServo");
        /*Servo rSleeve = hardwareMap.servo.get("rightsleeve");
        Servo lSleeve = hardwareMap.servo.get("leftSleeve");
        Servo outtake = hardwareMap.servo.get("outtakeServo");



        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake");
        DcMotor intake2 = hardwareMap.dcMotor.get("intake2");

        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");

        boolean latch = false;

        boolean modes = false;
        */








        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * 0.8; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.2; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


        if(-gamepad2.left_stick_y>0){
            claw.setPower(-0.5);
        }

        if(-gamepad2.left_stick_y==0){
            claw.setPower(-0.15);
        }


        if(-gamepad2.left_stick_y<0){
            claw.setPower(0.2);
        }
        if(gamepad2.right_trigger>0 && var==true && var2==true){
            cs2.setPosition(0.8);
            var=false;
            var2=false;
        }


        if(gamepad2.right_trigger>0 && var==true && var2==false){
            cs2.setPosition(0.5);
            var=false;
            var2=true;
        }

        if(gamepad2.right_trigger==0){
            var=true;
        }


        if(gamepad2.left_trigger>0 && othervar==true && othervar2==true){
            cs1.setPosition(0);
            othervar=false;
            othervar2=false;
        }


        if(gamepad2.left_trigger>0 && othervar==true && othervar2==false){
            cs1.setPosition(0.8);
            othervar=false;
            othervar2=true;
        }

        if(gamepad2.left_trigger==0){
            othervar=true;
        }


        if(gamepad2.dpad_up){
            plane.setPosition(1 );
        }

        if(-gamepad2.right_stick_y>0){
            rs.setPower(0.4);
            ls.setPower(-0.4);
        }

        if(gamepad2.right_stick_y==0){
            rs.setPower(0);
            ls.setPower(0);
        }

        if(-gamepad2.right_stick_y<0){
            rs.setPower(-0.4);
            ls.setPower(0.4);
        }

        }
    }
}