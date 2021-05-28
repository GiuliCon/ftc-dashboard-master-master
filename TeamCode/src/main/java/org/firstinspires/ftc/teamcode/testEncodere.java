package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "testEncodere", group = "Auto")

public class testEncodere extends LinearOpMode {
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(5000);
        rightFront.setTargetPosition(5000);
        leftRear.setTargetPosition(5000);
        rightRear.setTargetPosition(5000);

        leftFront.setPower(1);
        rightFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(1);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        while ( opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && leftFront.isBusy()){
            telemetry.addData("encoderLeftFront", leftFront.getCurrentPosition());
            telemetry.addData("encoderRightFront", rightFront.getCurrentPosition());
            telemetry.addData("encoderLeftRear", leftRear.getCurrentPosition());
            telemetry.addData("encoderRightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
