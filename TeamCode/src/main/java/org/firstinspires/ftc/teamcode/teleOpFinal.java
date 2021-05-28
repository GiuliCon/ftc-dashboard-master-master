package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@TeleOp(group = "advanced")
public class teleOpFinal extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        boolean launchMotorSS = false, launchServoSS = false, intakeMotorSS = false;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            if(gamepad1.a){
                if(!launchMotorSS){
                    drive.launchMotor.setPower(1);
                    launchMotorSS = true;
                }
                else {
                    drive.launchMotor.setPower(0);
                    launchMotorSS = false;
                }
            }
            if(gamepad1.b) {
                drive.launchServo.setPosition(1.0);
                sleep(300);
                drive.launchServo.setPosition(0);
                if(!launchServoSS) {
                    launchServoSS = true;
                }
                else {
                    launchServoSS = false;
                }
            }
            if(gamepad1.x) {
                if (!intakeMotorSS) {
                    drive.intakeMotor.setPower(1);
                    intakeMotorSS = true;
                } else {
                    drive.intakeMotor.setPower(0);
                    intakeMotorSS = false;
                }
            }
            drive.update();  
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}