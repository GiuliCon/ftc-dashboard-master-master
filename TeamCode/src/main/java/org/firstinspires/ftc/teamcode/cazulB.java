package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "CazB")
public class cazulB extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startpose = new Pose2d(-58, -50, 0);


        //region CazB
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


        //region StartCazB
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

    public void SingleFire() {
        drive.launchMotor.setPower(1);
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
}
