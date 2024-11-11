package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Field-Centric Mecanum Drive")
public class MecanumField extends OpMode {

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, outtakeMotorLeft, outtakeMotorRight;

    // Linear Slide Encoder Target Positions
    private final int LOW_POSITION = 0;
    private final int MEDIUM_POSITION = 1000; // Adjust these values based on testing
    private final int HIGH_POSITION = 2000;

    // Current target position
    private int targetPosition = LOW_POSITION;
    private boolean isAutomatedControl = false;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        outtakeMotorLeft = hardwareMap.get(DcMotor.class, "outLeft");
        outtakeMotorRight = hardwareMap.get(DcMotor.class, "outRight");

        // Set directions
        outtakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);  // Reverse if needed
        outtakeMotorRight.setDirection(DcMotor.Direction.FORWARD); // Forward direction

        // Set zero power behavior to brake for both slide motors
        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder and set motors to use encoders initially
        outtakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Driving logic (unchanged) ...

        // Encoder-based Linear Slide Control
        if (gamepad2.a) {
            targetPosition = LOW_POSITION;
            isAutomatedControl = true;
        } else if (gamepad2.b) {
            targetPosition = MEDIUM_POSITION;
            isAutomatedControl = true;
        } else if (gamepad2.y) {
            targetPosition = HIGH_POSITION;
            isAutomatedControl = true;
        }

        if (isAutomatedControl) {
            // Set target positions for both motors
            outtakeMotorLeft.setTargetPosition(targetPosition);
            outtakeMotorRight.setTargetPosition(targetPosition); // Use the same target for both

            // Switch to RUN_TO_POSITION mode
            outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to reach the target
            outtakeMotorLeft.setPower(0.5);
            outtakeMotorRight.setPower(0.5);

            // Check if motors have reached the target position
            if (!outtakeMotorLeft.isBusy() && !outtakeMotorRight.isBusy()) {
                // Stop motors and revert to manual mode
                outtakeMotorLeft.setPower(0);
                outtakeMotorRight.setPower(0);
                outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isAutomatedControl = false;
            }
        } else {
            // Manual Control for Linear Slides
            double manualPower = -gamepad2.left_stick_y * 0.6; // Scale power for manual control
            outtakeMotorLeft.setPower(manualPower);
            outtakeMotorRight.setPower(manualPower);
        }

        // Telemetry for debugging
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Slide Position", outtakeMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Position", outtakeMotorRight.getCurrentPosition());
        telemetry.addData("Automated Control", isAutomatedControl);
        telemetry.update();
    }
}

//Right encoder broken