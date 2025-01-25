package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Field Centric Mecanum Drive")
public class FieldCentricMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorSimple leftFront = hardwareMap.get(DcMotorSimple.class, "front_left_motor");
        DcMotorSimple leftBack = hardwareMap.get(DcMotorSimple.class, "back_left_motor");
        DcMotorSimple rightFront = hardwareMap.get(DcMotorSimple.class, "front_right_motor");
        DcMotorSimple rightBack = hardwareMap.get(DcMotorSimple.class, "back_right_motor");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            double lx = 0.6*gamepad1.left_stick_x;
            double ly = -0.6*gamepad1.left_stick_y;
            double rx = 0.6*gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * drivePower);
            leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * drivePower);
            rightFront.setPower(((-adjustedLy + adjustedLx + rx) / max) * drivePower);
            rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * drivePower);



            telemetry.addData("Heading",heading);
            telemetry.update();
        }
    }
}