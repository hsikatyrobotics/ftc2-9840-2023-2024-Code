package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.nio.file.WatchEvent;

@Autonomous
public class longRedAuto extends LinearOpMode {
    private DcMotorEx flm, blm, brm, frm;

    String position = "";

    OpenCvWebcam webcam1 = null;

    public void runtoPos(){
        flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() {
        flm = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        blm = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frm = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        brm = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        Servo cs1 = hardwareMap.servo.get("clawServo");
        Servo cs2 = hardwareMap.servo.get("clawServo2");
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

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

        class adiPipeline extends OpenCvPipeline{
            Mat YCbCr = new Mat();
            Mat leftCrop;
            Mat rightCrop;
            Mat middleCrop;
            double leftavgfin;
            double rightavgfin;
            double middleavgfin;
            Mat outPut = new Mat();
            Scalar rectColor = new Scalar(0, 0, 255.0);

            public Mat processFrame(Mat input){

                Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
                telemetry.addLine("pipeline running");

                Rect leftRect = new Rect(0, 250, 200,470);
                Rect rightRect =  new Rect(1080, 250, 200, 470);
                Rect middleRect = new Rect(201,0,800,200);


                input.copyTo(outPut);
                Imgproc.rectangle(outPut, leftRect, rectColor, 2);
                Imgproc.rectangle(outPut,rightRect,rectColor,2);
                Imgproc.rectangle(outPut,middleRect,rectColor, 2);

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

                if(leftavgfin > rightavgfin && leftavgfin > middleavgfin){
                    telemetry.addLine("Left");
                    position = "left";

                }

                if(rightavgfin > leftavgfin && rightavgfin> middleavgfin){
                    telemetry.addLine("Right");
                    position = "right";
                }

                if(middleavgfin > leftavgfin && middleavgfin > rightavgfin){
                    telemetry.addLine("middle");
                    position = "middle";
                }

                return(outPut);
            }
        }

        webcam1.setPipeline(new adiPipeline());

        cs1.setPosition(0.85);
        cs2.setPosition(0.2);





        waitForStart();


        if(position.equals("left")){
            flm.setTargetPosition(-1000);
            frm.setTargetPosition(-1000);
            blm.setTargetPosition(-1000);
            brm.setTargetPosition(-1000);
            runtoPos();
            flm.setPower(0.1);
            frm.setPower(0.1);
            blm.setPower(0.1);
            brm.setPower(0.1);

            while (opModeIsActive() && flm.getCurrentPosition() < flm.getTargetPosition()) {
                telemetry.addData("encoder-frontLeft", flm.getCurrentPosition());
                telemetry.addData("encoder-frontRight", frm.getCurrentPosition());
                telemetry.addData("encoder-backLeft", blm.getCurrentPosition());
                telemetry.addData("encoder-backRight", brm.getCurrentPosition());
                telemetry.update();
                idle();
            }

            frm.setPower(0);
            flm.setPower(0);
            blm.setPower(0);
            brm.setPower(0);



        }

        if(position.equals("middle")){

        }

        if(position.equals("right")){

        }





    }
}