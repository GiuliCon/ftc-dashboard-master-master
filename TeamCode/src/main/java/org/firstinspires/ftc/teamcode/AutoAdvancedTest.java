package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.advanced.PoseStorage;
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

@Autonomous(group = "advanced",preselectTeleOp = "TeleOpAlignWithPoint")
public class AutoAdvancedTest extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        // region CaseATrajectories
        Trajectory trajA1 = driveTrain.trajectoryBuilder(new Pose2d(-60, -55, Math.toRadians(180)))
//                .lineTo(new Vector2d(14, -55))
                .lineTo(new Vector2d(45, -55))
//                .addSpatialMarker(new Vector2d(5, -55), () -> {
//                    driveTrain.wobbleServo.setPosition(1);
//                })
                .build();

        Trajectory trajA2 = driveTrain.trajectoryBuilder(trajA1.end())
                .lineTo(new Vector2d(45, -15))
                .build();

        Trajectory trajA3 = driveTrain.trajectoryBuilder(trajA2.end())
                .lineTo(new Vector2d(-40, -20))
//                .addSpatialMarker(new Vector2d(-35, -20), () -> {
//                    driveTrain.wobbleServo.setPosition(0);
//                })
                .build();

        Trajectory trajA4 = driveTrain.trajectoryBuilder(trajA3.end())
                .lineToLinearHeading(new Pose2d(40, -60,  Math.toRadians(-90)))
                .build();

//        Trajectory trajA4 = driveTrain.trajectoryBuilder(trajA3.end())
//                .splineTo(new Vector2d(5, -70), Math.toRadians(352))
//                .build();
        //endregion

        // region CaseBTrajectories

        Trajectory trajB1 = driveTrain.trajectoryBuilder(new Pose2d(-60, -55, Math.toRadians(180)))
                .lineTo(new Vector2d(60,-35))
//                .addSpatialMarker(new Vector2d(20, -40),() -> {
//                    driveTrain.wobbleServo.setPosition(1);
//                })
                .build();
        Trajectory trajB2 = driveTrain.trajectoryBuilder(trajB1.end())
                .lineTo(new Vector2d(60,-5))
                .build();

        Trajectory trajB3 = driveTrain.trajectoryBuilder(trajB2.end())
                .lineTo(new Vector2d(-45, -5))
//                .addSpatialMarker(new Vector2d(-45, -5), () -> {
//                    driveTrain.wobbleServo.setPosition(0);
//                })
                .build();

        Trajectory trajB4 = driveTrain.trajectoryBuilder(trajB3.end())
                .lineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)))
//                .addSpatialMarker(new Vector2d(15, -10), () -> {
//                    driveTrain.wobbleServo.setPosition(1);
//                })
                .build();

        Trajectory trajB5 = driveTrain.trajectoryBuilder(trajB4.end())
                .lineTo(new Vector2d(50, -10))
                .build();

        //endregion

        //region CaseCTrajectories

        Trajectory trajC1 = driveTrain.trajectoryBuilder(new Pose2d(-60,-55,Math.toRadians(180)))
                .lineTo(new Vector2d(75,-55))
                .build();
        Trajectory trajC2 = driveTrain.trajectoryBuilder(trajC1.end())
                .lineTo(new Vector2d(-50,-55))
                .build();
        Trajectory trajC3 = driveTrain.trajectoryBuilder(trajC2.end())
                .lineToLinearHeading(new Pose2d(-50,-20, Math.toRadians(40)))
                .build();
        Trajectory trajC4 = driveTrain.trajectoryBuilder(trajC3.end())
                .splineTo(new Vector2d(0,-5),Math.toRadians(0))
//                .splineTo(new Vector2d(75,-40),Math.toRadians(0))
                .build();

        Trajectory trajC5 = driveTrain.trajectoryBuilder(trajC4.end())
                .splineTo(new Vector2d(75,-40),Math.toRadians(0))
                .build();

        Trajectory trajC6 = driveTrain.trajectoryBuilder(trajC5.end())
                .lineTo(new Vector2d(35,-35))
                .build();

//        for(int i = 0; i <= 10; i = i + 1)
//        {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//            sleep(50);
//        }

        //endregion

//        double x = 0;
//        while (opModeIsActive()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//            //x = x + 1;
//            // Don't burn CPU cycles busy-looping in this sample
//            //sleep(50);
//        }
//        int x = 0;
//        while (x<20) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//            x+=1;
//        }
        while (!isStarted()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }
        waitForStart();
//        int x = 0;
//        while (waitForStart();) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//            x+=1;
//        }

//        double x = 0;
//        while (opModeIsActive() && x < 10) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//            x = x + 1;
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }

//        while (opModeIsActive()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//            x = x + 1;
//
//            if (x == 50)
//                break;
//            // Don't burn CPU cycles busy-looping in this sample
//            //sleep(50);
//        }

        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE){
            driveTrain.followTrajectory(trajA1);
            driveTrain.followTrajectory(trajA2);
            driveTrain.followTrajectory(trajA3);
            driveTrain.followTrajectory(trajA4);
            PoseStorage.currentPose = trajA4.end();
//            PoseStorage.currentPose = trajA3.end();
        }

        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            driveTrain.followTrajectory(trajB1);
            driveTrain.followTrajectory(trajB2);
            driveTrain.followTrajectory(trajB3);
            driveTrain.followTrajectory(trajB4);
            driveTrain.followTrajectory(trajB5);
            PoseStorage.currentPose = trajB5.end();
        }

        else if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
            driveTrain.followTrajectory(trajC1);
            driveTrain.followTrajectory(trajC2);
            driveTrain.followTrajectory(trajC3);
            driveTrain.followTrajectory(trajC4);
            driveTrain.followTrajectory(trajC5);
            driveTrain.followTrajectory(trajC6);
            PoseStorage.currentPose = trajC6.end();

        }
        //driveTrain.wobbleServo.setPosition(1);
        for(int i=1;i<=100000;i++){
            telemetry.addData("i=",i);
            telemetry.update();
        }

    }

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

        final int FOUR_RING_THRESHOLD = 137;
        final int ONE_RING_THRESHOLD = 130;//todo: Modif ca sa il vada vedeti valorile initiale si vedeti diferentele si faceti raport si ar trebui sa mearga

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
