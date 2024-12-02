package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Field-Centric Mecanum Drive")
public class MecanumField extends OpMode {

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, outtakeMotorLeft, outtakeMotorRight;

    // Linear Slide Encoder Target Positions
    private final int LOW_POSITION = 0;
    private final int MEDIUM_POSITION = 1500; // Adjust these values based on testing
    private final int HIGH_POSITION = 3000;

    // Current target position
    private int targetPosition = LOW_POSITION;
    private boolean isAutomatedControl = false;


    // Define a small holding power to prevent sliding
    private final double HOLDING_POWER = 0;  // Adjust this value as needed

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
        //outtakeMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder and set motors to use encoders initially
        outtakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // outtakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Driving logic
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        double frontLeftPower = y + x + rotation;
        double frontRightPower = -y + x + rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        //_______________________________________________

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
           // outtakeMotorRight.setTargetPosition(targetPosition); // Use the same target for both

            // Switch to RUN_TO_POSITION mode
            outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           // outtakeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to reach the target
            outtakeMotorLeft.setPower(0.5);
           // outtakeMotorRight.setPower(0.5);

            // Check if motors have reached the target position
            if (!outtakeMotorLeft.isBusy()) {
                // Stop motors and revert to manual mode
                outtakeMotorLeft.setPower(HOLDING_POWER);
              //  outtakeMotorRight.setPower(HOLDING_POWER);
                outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             //   outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isAutomatedControl = false;
            }
        } else {
            // Manual Control for Linear Slides
            double manualPower = -gamepad2.left_stick_y * 0.6; // Scale power for manual control
            if (Math.abs(manualPower) > 0.05) {  // Apply only if joystick input is significant
                outtakeMotorLeft.setPower(manualPower);
                outtakeMotorRight.setPower(manualPower);
            } else {
                // Apply holding power when idle
                outtakeMotorLeft.setPower(HOLDING_POWER);
                outtakeMotorRight.setPower(HOLDING_POWER);
            }
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