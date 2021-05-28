 package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "advanced",preselectTeleOp = "TeleOpAlignWithPoint")
public class testControlHub extends LinearOpMode {
    SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory test1 = driveTrain.trajectoryBuilder(new Pose2d(-60, -50, 0))
                .forward(100)
                .build();
        waitForStart();


        driveTrain.followTrajectory(test1);
    }

}
