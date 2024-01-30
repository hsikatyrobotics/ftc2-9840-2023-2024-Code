package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import org.ejml.data.FSubmatrixD1;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;

import java.nio.file.WatchEvent;

@Config
@Autonomous
public class shortRedAuto extends OpMode {

    private PIDController controller;


    public static double p = 0.025, i= 0, d = 0.0007;
    public static double f=0.014;

    public static int target = 0;

    private final double ticks_in_degree = 1993.6 / 360;

    private DcMotorEx arm_motor;

    public int z = 0;


    private final double ticks_per_inch = 537.7 / 12;

    String position = "";

    DcMotorEx flm;
    DcMotorEx blm;
    DcMotorEx frm;
    DcMotorEx brm;

    Servo cs1;
    Servo cs2;

    int flp = 0;
    int frp = 0;
    int blp = 0;
    int brp = 0;


    public void motorsStop() {
        flm.setPower(0);
        frm.setPower(0);
        brm.setPower(0);
        blm.setPower(0);
    }

    public boolean openCVbool = true;

    public void resetEncoder() {
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void runtoPos() {
        frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updatePos() {
        flp = flm.getCurrentPosition();
        frp = frm.getCurrentPosition();
        brp = brm.getCurrentPosition();
        blp = blm.getCurrentPosition();
    }

    float inch = 0;

    OpenCvWebcam webcam1 = null;


    @Override
    public void init() {





        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "clawArmMotor");


        flm = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        blm = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frm = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        brm = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        cs1 = hardwareMap.get(Servo.class, "clawServo");
        cs2 = hardwareMap.get(Servo.class,"clawServo2");
        arm_motor = hardwareMap.get(DcMotorEx.class, "clawArmMotor");



        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        class adiPipeline extends OpenCvPipeline {
            Mat YCbCr = new Mat();
            Mat leftCrop;
            Mat rightCrop;
            Mat middleCrop;
            double leftavgfin;
            double rightavgfin;
            double middleavgfin;
            Mat outPut = new Mat();
            Scalar rectColor = new Scalar(0, 0, 0);

            public Mat processFrame(Mat input) {

                Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
                telemetry.addLine("pipeline running");

                Rect leftRect = new Rect(0, 0, 200, 470);
                Rect rightRect = new Rect(1080, 0, 200, 470);
                Rect middleRect = new Rect(250, 0, 750, 200);


                input.copyTo(outPut);
                Imgproc.rectangle(outPut, leftRect, rectColor, 2);
                Imgproc.rectangle(outPut, rightRect, rectColor, 2);
                Imgproc.rectangle(outPut, middleRect, rectColor, 2);

                leftCrop = YCbCr.submat(leftRect);
                rightCrop = YCbCr.submat(rightRect);
                middleCrop = YCbCr.submat(middleRect);

                Core.extractChannel(leftCrop, leftCrop, 2);
                Core.extractChannel(rightCrop, rightCrop, 2);
                Core.extractChannel(middleCrop, middleCrop, 2);

                Scalar leftavg = Core.mean(leftCrop);
                Scalar rightavg = Core.mean(rightCrop);
                Scalar middleavg = Core.mean(middleCrop);

                leftavgfin = leftavg.val[0];
                rightavgfin = rightavg.val[0];
                middleavgfin = middleavg.val[0];

                if (leftavgfin > rightavgfin && leftavgfin > middleavgfin) {
                    telemetry.addLine("Left");
                    position = "left";
                    openCVbool = false;
                    webcam1.stopStreaming();

                }

                if (rightavgfin > leftavgfin && rightavgfin > middleavgfin) {
                    telemetry.addLine("Right");
                    position = "right";
                    webcam1.stopStreaming();
                }

                if (middleavgfin > leftavgfin && middleavgfin > rightavgfin) {
                    telemetry.addLine("middle");
                    position = "middle";
                    webcam1.stopStreaming();
                }

                return (outPut);
            }
        }

        webcam1.setPipeline(new adiPipeline());

        cs1.setPosition(0.95);
        cs2.setPosition(0.2);

        telemetry.addData("flmPos", flm.getCurrentPosition());


    }


    public void loop(){
        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;


        /*if(position.equals("left")){
            position = "done";
            telemetry.addLine("starting");

            flm.setTargetPosition(1500);
            frm.setTargetPosition(-1500);
            blm.setTargetPosition(-1500);
            brm.setTargetPosition(1500);
            runtoPos();

            while(flm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            cs1.setPosition(0.5);

            flm.setTargetPosition(-1000);
            frm.setTargetPosition(1000);
            blm.setTargetPosition(-1000);
            brm.setTargetPosition(1000);
            runtoPos();

            while(blm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            arm_motor.setTargetPosition(400);
            arm_motor.setPower(0.25);

            flm.setTargetPosition(2000);
            frm.setTargetPosition(2000);
            blm.setTargetPosition(2000);
            brm.setTargetPosition(2000);
            runtoPos();

            while(brm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            cs2.setPosition(0.7);

            motorsStop();

            telemetry.addLine("radical");




        }

        if(position.equals("middle")){
            position="done";
            telemetry.addLine("starting");

            flm.setTargetPosition(-500);
            frm.setTargetPosition(500);
            blm.setTargetPosition(-500);
            brm.setTargetPosition(500);
            runtoPos();

            while(flm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            flm.setTargetPosition(1000);
            frm.setTargetPosition(1000);
            blm.setTargetPosition(1000);
            brm.setTargetPosition(1000);
            runtoPos();

            while(frm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            cs1.setPosition(0.5);

            flm.setTargetPosition(-1000);
            frm.setTargetPosition(1000);
            blm.setTargetPosition(-1000);
            brm.setTargetPosition(1000);
            runtoPos();

            while(flm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            arm_motor.setTargetPosition(300);
            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_motor.setPower(0.25);

            flm.setTargetPosition(2000);
            frm.setTargetPosition(2000);
            blm.setTargetPosition(2000);
            brm.setTargetPosition(2000);
            runtoPos();

            while(flm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            cs2.setPosition(0.5);



        }

        if(position.equals("right")){
            position = "done";

            telemetry.addLine("Running");

            flm.setTargetPosition(500);
            frm.setTargetPosition(-500);
            blm.setTargetPosition(-500);
            brm.setTargetPosition(500);
            runtoPos();

            while(flm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();


            flm.setTargetPosition(-500);
            frm.setTargetPosition(500);
            blm.setTargetPosition(-500);
            brm.setTargetPosition(500);
            runtoPos();

            while(frm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            flm.setTargetPosition(1000);
            frm.setTargetPosition(1000);
            blm.setTargetPosition(1000);
            brm.setTargetPosition(1000);
            runtoPos();

            while(blm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            cs1.setPosition(0.5);

            flm.setTargetPosition(-300);
            frm.setTargetPosition(-300);
            blm.setTargetPosition(-300);
            brm.setTargetPosition(-300);
            runtoPos();

            while(blm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            flm.setTargetPosition(-500);
            frm.setTargetPosition(500);
            blm.setTargetPosition(-500);
            brm.setTargetPosition(500);
            runtoPos();


            while(brm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }

            motorsStop();
            resetEncoder();

            arm_motor.setTargetPosition(300);
            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_motor.setPower(0.4);


            flm.setTargetPosition(2000);
            frm.setTargetPosition(2000);
            blm.setTargetPosition(2000);
            brm.setTargetPosition(2000);
            runtoPos();

            while(blm.isBusy()){
                flm.setPower(0.25);
                frm.setPower(0.25);
                blm.setPower(0.25);
                brm.setPower(0.25);
            }


            cs2.setPosition(0.5);


        }*/

        flm.setTargetPosition(1000);
        frm.setTargetPosition(1000);
        blm.setTargetPosition(1000);
        brm.setTargetPosition(1000);
        runtoPos();


        while(flm.isBusy()){
            flm.setPower(0.25);
            frm.setPower(0.25);
            blm.setPower(0.25);
            brm.setPower(0.25);
        }

        motorsStop();
        resetEncoder();

        flm.setTargetPosition(-500);
        frm.setTargetPosition(500);
        blm.setTargetPosition(-500);
        brm.setTargetPosition(500);
        runtoPos();

        while(blm.isBusy()){
            flm.setPower(0.25);
            frm.setPower(0.25);
            blm.setPower(0.25);
            brm.setPower(0.25);
        }

        motorsStop();
        resetEncoder();


        flm.setTargetPosition(1500);
        frm.setTargetPosition(1500);
        blm.setTargetPosition(1500);
        brm.setTargetPosition(1500);
        runtoPos();

        while(blm.isBusy()){
            flm.setPower(0.25);
            frm.setPower(0.25);
            blm.setPower(0.25);
            brm.setPower(0.25);
        }

        motorsStop();
        resetEncoder();
        cs1.setPosition(0);
        cs2.setPosition(0);



    }
}
