package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mecanum Drive")
public class MecanumField extends OpMode {

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, vertical, horizontal;
    //CRServo arm;
    Servo claw, bucket, arm;

    IMU imu;


    // Linear Slide Encoder Target Positions
    private final int LOW_POSITION = 0;
    private final int MEDIUM_POSITION = 1500;
    private final int HIGH_POSITION = 3000;

    private int targetPosition = LOW_POSITION;
    private boolean isAutomatedVertical = false;
    private boolean isAutomatedHorizontal = false;

    private boolean prevClaw, prevArm, prevBucket = false;
    private boolean clawClosed, armClosed, bucketClosed = false;


    private final double HOLDING_POWER = 0;  // Adjust as needed


    //Automated teleop variables
    private boolean isPresetActive = false;
    private final int PRESET_VERTICAL_POSITION = 0;
    private final int PRESET_HORIZONTAL_POSITION = 0;
    private final double PRESET_BUCKET_POSITION = 0;
    private final double PRESET_ARM_POSITION = 0.05;
    private final double PRESET_CLAW_POSITION = 0.7;

    @Override
    public void init() {
        // Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        vertical = hardwareMap.get(DcMotor.class, "outLeft");
        horizontal = hardwareMap.get(DcMotor.class, "outRight");

        vertical.setDirection(DcMotor.Direction.REVERSE);
        horizontal.setDirection(DcMotor.Direction.REVERSE);

        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servos
        arm = hardwareMap.get(Servo.class, "arm");
        bucket = hardwareMap.get(Servo.class, "bucket");
        claw = hardwareMap.get(Servo.class, "claw");

        arm.setPosition(0.05);
        bucket.setPosition(0.7);
        claw.setPosition(0.7);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void loop() {

        boolean isSequenceActive = false;
        boolean motorsAtZero = false;
        boolean clawMoved = false;
        boolean armMoved = false;
        boolean clawFinalMoved = false;
        boolean verticalFinalMoved = false;
        long clawTimer = 0;

        boolean currClaw = gamepad2.a;
        boolean currArm = gamepad2.x;
        boolean currBucket = gamepad2.b;
        // Basic mecanum drive logic

        // Driving logic
        double lx = 0.6 * gamepad1.left_stick_x;
        double ly = -0.6 * gamepad1.left_stick_y;
        double rx = 0.6 * gamepad1.right_stick_x;

        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);


        double heading = -1 * imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

        frontLeftMotor.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
        backLeftMotor.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
        frontRightMotor.setPower(((-adjustedLy + adjustedLx + rx) / max) * drivePower);
        backRightMotor.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);


        //-----------------------------------------------------------------------------------------

        // Servos


        if (currArm && !prevArm) {
            if (armClosed) {
                arm.setPosition(0.55);
            } else {

                arm.setPosition(0.05);
            }

            armClosed = !(armClosed);

        }


        //bucket

        if (currBucket && !prevBucket) {
            if (bucketClosed) {
                bucket.setPosition(0);
            } else {

                bucket.setPosition(0.7);
            }

            bucketClosed = !(bucketClosed);

        }


        //claw (NOT continuous)
        //0.9=open, 0.7=closed
        if (currClaw && !prevClaw) {
            if (clawClosed) {
                claw.setPosition(0.7);
            } else {

                claw.setPosition(0.9);
            }

            clawClosed = !(clawClosed);

        }


            // Encoder-based Linear Slide Control
            if (gamepad1.a) {
                targetPosition = LOW_POSITION; // Vertical slide 0
                isAutomatedVertical = true;
            } else if (gamepad1.b) {
                targetPosition = MEDIUM_POSITION; // Vertical slide 1st tier
                isAutomatedVertical = true;
            } else if (gamepad1.y) {
                targetPosition = HIGH_POSITION; // Vertical slide 2nd tier
                isAutomatedVertical = true;
            } else if (gamepad1.x) {
                targetPosition = LOW_POSITION; // Horizontal slide 0
                isAutomatedHorizontal = true;
            }


            // Vertical Automated + Manual Control
            if (isAutomatedVertical) {
                vertical.setTargetPosition(targetPosition);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertical.setPower(-0.5);


                if (!vertical.isBusy()) {
                    vertical.setPower(HOLDING_POWER);
                    vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    isAutomatedVertical = false;
                }
            } else {
                if (gamepad2.right_bumper) {


                    if (horizontal.getCurrentPosition() > 3080) {
                        vertical.setPower(0);
                    } else {
                        vertical.setPower(0.25);
                    }
                } else if (gamepad2.left_bumper) {
                    vertical.setPower(-0.25);
                } else {
                    vertical.setPower(HOLDING_POWER); // Adjust holding power if necessary
                }
            }


            // Horizontal Automated + Manual Control
            if (isAutomatedHorizontal) {
                horizontal.setTargetPosition(targetPosition);
                horizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontal.setPower(-0.5);


                if (!horizontal.isBusy()) {
                    horizontal.setPower(HOLDING_POWER);
                    horizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    isAutomatedHorizontal = false;
                }
            } else {
                double horizontalPower = -gamepad2.left_stick_y * 0.6;
                if (Math.abs(horizontalPower) > 0.05) {
                    horizontal.setPower(horizontalPower * 0.8);


                }
                //new code posible fix to slide issue
                else {
                    horizontal.setPower(0);
                }
            }


                prevClaw = currClaw;
                prevArm = currArm;
                prevBucket = currBucket;


                telemetry.addData("Vertical Slide", vertical.getCurrentPosition());
                telemetry.addData("Horizontal Slide", horizontal.getCurrentPosition());
                telemetry.addData("Arm Position", arm.getPosition());
                telemetry.addData("Bucket Position", bucket.getPosition());
                telemetry.addData("Claw Position", claw.getPosition());
                telemetry.addData("Heading", heading);
                telemetry.update();


        }
    }
