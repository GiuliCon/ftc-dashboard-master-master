package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "CazC")
public class cazulC extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(-58, -50, 0);


        //region CazC
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


        //region StartCazC
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

    public void SingleFire() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.launchMotor.setPower(1);
        sleep(100);
        drive.launchServo.setPosition(1);
        sleep(100);
        drive.launchServo.setPosition(0);
        sleep(200);
        drive.launchMotor.setPower(0);
    }

    public void Burst() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
}
