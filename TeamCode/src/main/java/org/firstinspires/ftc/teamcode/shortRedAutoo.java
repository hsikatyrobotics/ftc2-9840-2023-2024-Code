package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker;
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.SpatialMarker;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;

import org.ejml.data.FSubmatrixD1;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
public class shortRedAutoo extends OpMode {

    private PIDController controller;


    public static double p = 0.025, i= 0, d = 0.0007;
    public static double f=0.014;

    public static int target = 0;

    private final double ticks_in_degree = 1993.6 / 360;

    private DcMotorEx arm_motor;

    public int z = 0;


    private final double ticks_per_inch = 537.7 / 12;

    String position = "";

    DcMotor flm = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor blm = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frm = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor brm = hardwareMap.dcMotor.get("backRightMotor");



    DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");


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

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "clawArmMotor");



        flm = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        blm = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frm = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        brm = hardwareMap.get(DcMotorEx.class, "backRightMotor");
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
            Mat HSV = new Mat();
            Mat leftCrop;
            Mat rightCrop;
            Mat middleCrop;
            double leftavgfin;
            double rightavgfin;
            double middleavgfin;
            double PERCENT_COLOR_THRESHOLD = 0.4;

            Mat outPut = new Mat();

            public Mat processFrame(Mat input) {

                Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
                telemetry.addLine("pipeline running");
                Scalar lowHsv = new Scalar(355, 75, 100);
                Scalar highHsv = new Scalar(355, 90, 100);

                Rect leftRect = new Rect(0, 0, 200, 470);
                Rect rightRect = new Rect(1080, 0, 200, 470);
                Rect middleRect = new Rect(250, 0, 750, 200);

                Core.inRange(HSV, lowHsv, highHsv, HSV);


                input.copyTo(outPut);

                leftCrop = HSV.submat(leftRect);
                rightCrop = HSV.submat(rightRect);
                middleCrop = HSV.submat(middleRect);

                double leftValue = Core.sumElems(leftCrop).val[0] / leftRect.area() / 255;
                double rightValue = Core.sumElems(rightCrop).val[0] / rightRect.area() / 255;
                double middleValue = Core.sumElems(middleCrop).val[0] / middleRect.area() / 255;

                leftCrop.release();
                rightCrop.release();
                middleCrop.release();

                telemetry.addData("Left Raw Value", (int) Core.sumElems(leftCrop).val[0]);
                telemetry.addData("Right Raw Value", (int) Core.sumElems(rightCrop).val[0]);
                telemetry.addData("Middle Raw Value", (int) Core.sumElems(middleCrop).val[0]);
                telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
                telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
                telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

                boolean left = leftValue > PERCENT_COLOR_THRESHOLD;
                boolean right = rightValue > PERCENT_COLOR_THRESHOLD;
                boolean middle = middleValue > PERCENT_COLOR_THRESHOLD;

                if(left && right && middle){
                    position = "not found";
                }if(left){
                    position = "left";
                    webcam1.stopStreaming();
                }if(right){
                    position = "right";
                    webcam1.stopStreaming();
                }if(middle){
                    position = "middle";
                    webcam1.stopStreaming();
                }



                return (outPut);
            }
        }

        webcam1.setPipeline(new adiPipeline());


        telemetry.addData("flmPos", flm.getCurrentPosition());



    }

    @Override
    public void loop() {


        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;


        if(position.equals("right")){
            TrajectorySequence right1 = drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                    .splineTo(new Vector2d(15,-34), Math.toRadians(90))
                    .build();


            drive.followTrajectorySequence(right1);

            //deposit purple pixel

            TrajectorySequence right2 = drive.trajectorySequenceBuilder(new Pose2d(15,-34, Math.toRadians(90)))
                    .back(5)
                    .turn(Math.toRadians(90))
                    .back(30)
                    .build();

            drive.followTrajectorySequence(right2);

            //deposit yellow pixel



        }


        if(position.equals("middle")){
            TrajectorySequence middle1 = drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                    .splineTo(new Vector2d(32,-34), Math.toRadians(180))
                    .build();


            drive.followTrajectorySequence(middle1);

            //deposit purple pixel

            TrajectorySequence middle2 = drive.trajectorySequenceBuilder(new Pose2d(15,-34, Math.toRadians(90)))
                    .back(15)
                    .build();

            drive.followTrajectorySequence(middle2);

            //deposit yellow pixel



        }


        if(position.equals("left")){

            TrajectorySequence left1 = drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .build();

            drive.followTrajectorySequence(left1);

            //deposit purple pixel


            TrajectorySequence left2 =  drive.trajectorySequenceBuilder(new Pose2d(10, -34, Math.toRadians(180)))
                    .back(37)
                    .build();

            drive.followTrajectorySequence(left2);

            //deposit yellow pixel


        }














    }
}
