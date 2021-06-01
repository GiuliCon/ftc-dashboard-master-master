package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "CazA")
public class cazulA extends LinearOpMode {
//private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-58, -50, 0);

        waitForStart();

        //region CazA
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

        //region StartCazA

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


    public void SingleFire() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.launchMotor.setPower(1);
        sleep(1000);
        drive.launchServo.setPosition(-1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(200);
        drive.launchMotor.setPower(0);
    }
}

