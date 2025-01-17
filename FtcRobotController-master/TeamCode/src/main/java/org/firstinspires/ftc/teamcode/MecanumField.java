package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Field-Centric Mecanum Drive")
public class MecanumField extends OpMode {

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, outtakeMotorLeft, outtakeMotorRight;
    Servo claw, wrist;

    BNO055IMU imu; // Declare the IMU

    // Linear Slide Encoder Target Positions
    private final int LOW_POSITION = 0;
    private final int MEDIUM_POSITION = 1500;
    private final int HIGH_POSITION = 3000;

    private int targetPosition = LOW_POSITION;
    private boolean isAutomatedVertical = false;

    private boolean isAutomatedHorizontal = false;

    private final double HOLDING_POWER = 0;  // Adjust as needed

    @Override
    public void init() {
        // Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        outtakeMotorLeft = hardwareMap.get(DcMotor.class, "outLeft");
        outtakeMotorRight = hardwareMap.get(DcMotor.class, "outRight");

        outtakeMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotorRight.setDirection(DcMotor.Direction.FORWARD);

        outtakeMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servos
        claw = hardwareMap.get(Servo.class,"claw");
        wrist = hardwareMap.get(Servo.class,"wrist");

        // IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; // Use radians for calculations
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        // Field-centric driving logic
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x * 0.6;

        // Get the robot's heading from the IMU
        Orientation angles = imu.getAngularOrientation();
        double heading = angles.firstAngle;

        // Rotate joystick input by IMU heading
        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double frontLeftPower = fieldY + fieldX + rotation;
        double frontRightPower = -fieldY + fieldX + rotation;
        double backLeftPower = fieldY - fieldX + rotation;
        double backRightPower = fieldY + fieldX - rotation;

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

        //-----------------------------------------------------------------------------------------

        // Servos
        if (gamepad2.a) {
            claw.setPosition(0.8);
        } else {
            claw.setPosition(0);
        }


        if (gamepad2.b) {
            wrist.setPosition(0.8);
        } else {
            wrist.setPosition(0);
        }


        // Encoder-based Linear Slide Control
        if (gamepad1.a) {
            targetPosition = LOW_POSITION; //Vertical slide 0
            isAutomatedVertical = true;
        } else if (gamepad1.b) {
            targetPosition = MEDIUM_POSITION; //Vertical slide 1st tier
            isAutomatedVertical = true;
        } else if (gamepad1.y) {
            targetPosition = HIGH_POSITION; //Vertical slide 2nd tier
            isAutomatedVertical = true;
        } else if(gamepad1.x) {
            targetPosition = LOW_POSITION; //Horizontal slide 0
            isAutomatedHorizontal = true;
        }

        //Vertical Automated + Manual Control
        if (isAutomatedVertical) {
            outtakeMotorLeft.setTargetPosition(targetPosition);
            outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeMotorLeft.setPower(0.5);

            if (!outtakeMotorLeft.isBusy()) {
                outtakeMotorLeft.setPower(HOLDING_POWER);
                outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isAutomatedVertical = false;
            }
        } else {

            if (gamepad2.right_bumper) {
                outtakeMotorLeft.setPower(0.5);
            } else if (gamepad2.left_bumper) {
                outtakeMotorLeft.setPower(-0.5);
            } else {
                outtakeMotorLeft.setPower(HOLDING_POWER); //adjust holding power if necessary
            }
        }


        //Horizontal Automated + Manual Control
        if (isAutomatedHorizontal) {
            outtakeMotorRight.setTargetPosition(targetPosition);
            outtakeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeMotorRight.setPower(0.5);

            if (!outtakeMotorRight.isBusy()) {
                outtakeMotorRight.setPower(HOLDING_POWER);
                outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                isAutomatedHorizontal = false;
            }
        } else {
            double horizontalPower = -gamepad2.left_stick_y * 0.6;
            if (Math.abs(horizontalPower) > 0.05) {
                outtakeMotorRight.setPower(horizontalPower);
            } else {
                outtakeMotorRight.setPower(HOLDING_POWER); //adjust holding power if necessary
            }
        }

        telemetry.addData("Heading (radians)", heading);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Slide Position", outtakeMotorLeft.getCurrentPosition());
        telemetry.addData("Automated Vertical", isAutomatedVertical);
        telemetry.addData("Horizontal Position", outtakeMotorRight.getCurrentPosition());
        telemetry.addData("Automated Horizontal", isAutomatedHorizontal);
        telemetry.update();
    }
}
