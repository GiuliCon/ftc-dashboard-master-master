package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous (name="TestEncodereRev")
public class testEncidereRev extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-62, 0, 0);


        Encoder parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        Encoder perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

        drive.setPoseEstimate(startpose);

        Trajectory traj1 = drive.trajectoryBuilder(startpose)
                .lineTo(new Vector2d(10, 0))
                .build();
        waitForStart();

        parallelEncoder.resetPosition();
        perpendicularEncoder.resetPosition();

        telemetry.addData("parallel Encoder", parallelEncoder.getCurrentPosition());
        telemetry.addData("perpendicular Encoder", perpendicularEncoder.getCurrentPosition());
        telemetry.update();

        drive.followTrajectory(traj1);
        while(opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("parallel Encoder", parallelEncoder.getCurrentPosition());
            telemetry.addData("perpendicular Encoder", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}