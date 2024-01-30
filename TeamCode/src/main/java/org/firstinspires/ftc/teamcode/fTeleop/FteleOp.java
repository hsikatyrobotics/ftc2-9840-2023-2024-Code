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



    private VisionPortal myVisionPortal;

    public int z = 0;


    boolean latch_variable_1 = false;




    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor Lhang = hardwareMap.dcMotor.get("leftHang");
        DcMotor Rhang = hardwareMap.dcMotor.get("rightHang");

        DcMotor arm_motor = hardwareMap.dcMotor.get("clawArmMotor");

        Lhang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo clawServo2 = hardwareMap.servo.get("clawServo2");
        Servo planeServo = hardwareMap.servo.get("planeServo");


        Lhang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lhang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
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









            if(gamepad2.a){
                arm_motor.setTargetPosition(95);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setPower(0.5);

            }


            if(gamepad2.b){
                arm_motor.setTargetPosition(80);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setPower(0.5);
            }

            if(gamepad2.x){
                arm_motor.setTargetPosition(66);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setPower(0.5);
            }

            if(gamepad2.y){
                arm_motor.setTargetPosition(55);
                arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_motor.setPower(0.5);
            }



            if(gamepad2.right_trigger>0){
                clawServo.setPosition(0.85);
            }



            if(gamepad2.left_trigger>0){
                clawServo2.setPosition(0);
            }

            if(gamepad2.right_bumper){
                clawServo.setPosition(0.6);
            }

            if(gamepad2.left_bumper){
                clawServo2.setPosition(0.9);
            }

            if(gamepad2.dpad_right){
                planeServo.setPosition(1.5);
            }





            if(-gamepad2.left_stick_y>0){
                arm_motor.setPower(0.4);
            }else{
                arm_motor.setPower(0.15);
            }

            if(-gamepad2.left_stick_y<0) {
                arm_motor.setPower(-0.15);
            }



            if(gamepad2.dpad_up){
                Lhang.setTargetPosition(30000);
                Lhang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lhang.setPower(1);
                Rhang.setTargetPosition(30000);
                Rhang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rhang.setPower(1);

            }

            if(gamepad2.dpad_down){
                Lhang.setTargetPosition(0);
                Lhang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lhang.setPower(1);
                Rhang.setTargetPosition(0);
                Rhang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rhang.setPower(1);
            }

        }
    }
}