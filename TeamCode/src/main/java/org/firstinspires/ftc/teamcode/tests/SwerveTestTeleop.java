package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.SwerveDrivetrain;

@TeleOp(name="Basic Swerve Movement Teleop", group="Swerve Drivetrain Tests")
public class SwerveTestTeleop extends LinearOpMode {

    SwerveDrivetrain swerve = new SwerveDrivetrain();
    DcMotor intake = null;

    private DcMotor intake = null;

    @Override
    public void runOpMode() {

        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            intake = null;
        }

        swerve.init(hardwareMap);
        telemetry.addLine("swerve initialization was a success!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("swerve drive has started");
            swerve.drive(gamepad1, telemetry);

            if (intake != null) {
                if (gamepad1.dpad_right) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
            }

            telemetry.update();

            idle();
        }
    }
}