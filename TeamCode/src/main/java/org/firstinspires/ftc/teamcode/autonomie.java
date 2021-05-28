package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(group = "advanced", name = "Autonomie1")
public class autonomie extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startpose = new Pose2d(-58, -50, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //TODO: --- A ---
        //region InitA
        Trajectory trajA1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(new Pose2d(5, -56, Math.toRadians(-120)))
                .build();

        Trajectory trajA2 = drive.trajectoryBuilder(trajA1.end())
                .lineToLinearHeading(new Pose2d(-50, -42, Math.toRadians(0)))
                .build();

        Trajectory trajA3 = drive.trajectoryBuilder(trajA2.end())
                .lineToLinearHeading(new Pose2d(0, -56, Math.toRadians(-120)))
                .build();

        Trajectory trajA4 = drive.trajectoryBuilder(trajA3.end())
                .lineToLinearHeading(new Pose2d(0, -19, Math.toRadians(20)))
                .build();

        Trajectory parkA = drive.trajectoryBuilder(new Pose2d(0, -19, Math.toRadians(6.25)))
                .forward(10)
                .build();
        //endregion

        //TODO: --- B ---
        //region InitB
        Trajectory trajB1 = drive.trajectoryBuilder(startpose)
                .lineToLinearHeading(new Pose2d(30, -50, 0))
                .build();

        Trajectory trajB2 = drive.trajectoryBuilder(trajB1.end())
                .lineToLinearHeading(new Pose2d(33, -37, Math.toRadians(-45)))
                .build();

        Trajectory trajB3 = drive.trajectoryBuilder(trajB2.end())
                .lineToLinearHeading(new Pose2d(-5, -25, 0))
                .build();

        Trajectory trajB4 = drive.trajectoryBuilder(new Pose2d(-5, -25, Math.toRadians(35)))
                .lineToLinearHeading(new Pose2d(-17, -32, Math.toRadians(35)))
                .build();

        Trajectory trajB5 = drive.trajectoryBuilder(trajB4.end())
                .lineToLinearHeading(new Pose2d(-5, -25, 0))
                .build();

        Trajectory trajB6 = drive.trajectoryBuilder(trajB5.end())
                .lineToLinearHeading(new Pose2d(-35, -25, Math.toRadians(90)))
                .build();

        Trajectory trajB7 = drive.trajectoryBuilder(trajB6.end())
                .lineToLinearHeading(new Pose2d(20, -35, Math.toRadians(-90)))
                .build();

        Trajectory parkB = drive.trajectoryBuilder(trajB7.end())
                .strafeRight(10)
                .build();
        //endregion

        //TODO: --- C ---
        //region InitC
        Trajectory trajC1 = drive.trajectoryBuilder(startpose)
                .lineTo(new Vector2d(0, -50))
                .build();

        Trajectory trajC2 = drive.trajectoryBuilder(trajC1.end())
                .lineToLinearHeading(new Pose2d(0, -23, Math.toRadians(20)))
                .build();

        Trajectory trajC3 = drive.trajectoryBuilder(new Pose2d(0, -23, Math.toRadians(6.25)))
                .lineToLinearHeading(new Pose2d(55, -60, Math.toRadians(-90)))
                .build();

        Trajectory trajC4 = drive.trajectoryBuilder(trajC3.end())
                .lineToLinearHeading(new Pose2d(0, -36, 0))
                .build();

        Trajectory trajC5 = drive.trajectoryBuilder(trajC4.end())
                .lineTo(new Vector2d(-13, -36))
                .build();

        Trajectory trajC6 = drive.trajectoryBuilder(trajC5.end())
                .lineTo(new Vector2d(0, -36))
                .build();

        Trajectory trajC7 = drive.trajectoryBuilder(trajC6.end())
                .lineTo(new Vector2d(-17, -36))
                .build();

        Trajectory trajC8 = drive.trajectoryBuilder(trajC7.end())
                .lineToLinearHeading(new Pose2d(-35, -24, Math.toRadians(90)))
                .build();

        Trajectory trajC9 = drive.trajectoryBuilder(trajC8.end())
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(0)))
                .build();

        Trajectory trajC10 = drive.trajectoryBuilder(trajC9.end())
                .lineToLinearHeading(new Pose2d(50, -60, Math.toRadians(-90)))
                .build();

        Trajectory parkC = drive.trajectoryBuilder(trajC10.end())
                .lineToLinearHeading(new Pose2d(10, -50, Math.toRadians(0)))
                .build();
        //endregion

        while (!isStarted()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }
        waitForStart();

        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE){
            //region CazulA
            drive.followTrajectory(trajA1);
            //TODO: arm-ul va lasa wobble-ul
            drive.followTrajectory(trajA2);
            //TODO: arm-ul va lua al doilea wobblegoal
            drive.followTrajectory(trajA3);
            //TODO: arm-ul va lasa al doilea wobblegoal
            drive.followTrajectory(trajA4);
            // robotul trage in primul powershot
            SingleFire();
            drive.turn(Math.toRadians(12.5));
            // robotul trage in al doilea powershot
            SingleFire();
            drive.turn(Math.toRadians(6.25));
            // robotul trage in al treilea powershot
            SingleFire();
            drive.followTrajectory(parkA);

            //endregion
        }

        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            //region CazulB
            drive.followTrajectory(trajB1);
            drive.followTrajectory(trajB2);
            //TODO: arm-ul va lasa wobble-ul
            drive.followTrajectory(trajB3);
            //Robotul trage toate 3 disc-urile
            FullAuto();
            drive.turn(Math.toRadians(35));
            drive.intakeMotor.setPower(1);
            sleep(1000);
            drive.followTrajectory(trajB4);
            //Robotul ia ring-ul de pe jos
            drive.intakeMotor.setPower(0);
            drive.followTrajectory(trajB5);
            //Robotul trage ring-ul
            SingleFire();
            drive.followTrajectory(trajB6);
            //TODO: arm-ul ia al doilea wobble
            drive.followTrajectory(trajB7);
            //TODO: arm-ul lasa al doilea wobble
            drive.followTrajectory(parkB);
            //endregion
        }

        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
            //region CazulC
            drive.followTrajectory(trajC1);
            drive.followTrajectory(trajC2);
            SingleFire();
            drive.turn(Math.toRadians(12.5));
            SingleFire();
            drive.turn(Math.toRadians(6.25));
            SingleFire();
            drive.followTrajectory(trajC3);
            //TODO: arm-ul lasa al doilea wobble
            drive.followTrajectory(trajC4);
            drive.intakeMotor.setPower(1);
            drive.followTrajectory(trajC5);
            sleep(1000);
            drive.intakeMotor.setPower(0);
            drive.followTrajectory(trajC6);
            Burst();
            drive.intakeMotor.setPower(1);
            drive.followTrajectory(trajC7);
            sleep(1000);
            drive.intakeMotor.setPower(0);
            drive.followTrajectory(trajC8);
            //TODO: arm-ul ia cel de al doilea wobble
            drive.followTrajectory(trajC9);
            Burst();
            drive.followTrajectory(trajC10);
            //TODO: arm-ul lasa cel de al doilea wobble
            drive.followTrajectory(parkC);
            //endregion
        }

    }

    public void SingleFire() {
        drive.launchMotor.setPower(1);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(200);
        drive.launchMotor.setPower(0);
    }

    public void Burst() {
        drive.launchMotor.setPower(1);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(200);
        drive.launchMotor.setPower(0);
    }

    public void FullAuto() {
        drive.launchMotor.setPower(1);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(200);
        drive.launchMotor.setPower(0);

    }
//    public void CaseA() {
//
//    }
//
//    public void CaseB() {
//
//    }
//
//    public void CaseC() {
//
//    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         *
         * ASTA II IMPORTANT
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(5,70); //todo: De aci setam pipeline

        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 70;

        final int FOUR_RING_THRESHOLD = 134;
        final int ONE_RING_THRESHOLD = 129;//todo: Modif ca sa il vada vedeti valorile initiale si vedeti diferentele si faceti raport si ar trebui sa mearga

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization--
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
