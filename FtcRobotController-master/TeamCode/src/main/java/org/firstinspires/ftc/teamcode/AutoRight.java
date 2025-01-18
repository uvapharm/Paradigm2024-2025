package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoRight")
public class AutoRight extends LinearOpMode {
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    int targetPos = 1000;

    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        if (opModeIsActive()) {

            frontLeftMotor.setTargetPosition(targetPos);
            frontRightMotor.setTargetPosition(targetPos);
            backLeftMotor.setTargetPosition(targetPos);
            backRightMotor.setTargetPosition(targetPos);
            // outtakeMotorRight.setTargetPosition(targetPosition); // Use the same target for both

            // Switch to RUN_TO_POSITION mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // outtakeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to reach the target
            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            // outtakeMotorRight.setPower(0.5);

            // Check if motors have reached the target position
            if (!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() && !backLeftMotor.isBusy() && !backRightMotor.isBusy()) {
                // Stop motors and revert to manual mode
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                //  outtakeMotorRight.setPower(HOLDING_POWER);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }

    }
}