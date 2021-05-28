package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous (name="TestEncodereRev")
public class testEncidereRev extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-58, -50, 0);

        Encoder parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        Encoder perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

        drive.setPoseEstimate(startpose);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("parallel Encoder", parallelEncoder.getCurrentPosition());
            telemetry.addData("perpendicular Encoder", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
            telemetry.addData("x:    ", poseEstimate.getX());
            telemetry.addData("y:    ", poseEstimate.getY());
            telemetry.addData("head: ", poseEstimate.getHeading());
        }
    }
}