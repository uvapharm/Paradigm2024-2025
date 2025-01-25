package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoForward")
public class AutoRight extends LinearOpMode {

    // Declare motor variables
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    private double time = 0;
    private double timeint = 0;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the start of the autonomous period
        waitForStart();

        frontLeftMotor.setPower(0.4);
        frontRightMotor.setPower(0.4);



        sleep(4000);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);



    }
}

        /*
        timeint = getRuntime();

        while (time < 1000+timeint) {
            frontLeftMotor.setPower(0.3);
            frontRightMotor.setPower(0.3);
            time = getRuntime();
        }
        time = getRuntime();
        if (time > 10000+timeint){

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            time = getRuntime();


        }





    }
}

        if (opModeIsActive()) {
            // Set the target position for all motors
            int targetPos = 1000; // Adjust based on your robot's wheel circumference and encoder counts
            frontLeftMotor.setTargetPosition(targetPos);
            frontRightMotor.setTargetPosition(targetPos);


            // Switch to RUN_TO_POSITION mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply power to move the robot forward
            double motorPower = 0.5; // Adjust as needed
            frontLeftMotor.setPower(motorPower);
            frontRightMotor.setPower(motorPower);


            // Wait for approximately 3 seconds or until the motors reach their target
            while (opModeIsActive() &&
                    (frontLeftMotor.isBusy() || frontRightMotor.isBusy())) {
                telemetry.addData("Status", "Motors running to target");
                telemetry.update();
            }

            // Stop all motors
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);


            // Revert motors to manual mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }



         */




