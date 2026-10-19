package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.SwerveDrivetrain;

@TeleOp(name="Basic Swerve Movement Teleop", group="Swerve Drivetrain Tests")
public class SwerveTestTeleop extends LinearOpMode {

    SwerveDrivetrain swerve = new SwerveDrivetrain();

    @Override
    public void runOpMode() {

        swerve.init(hardwareMap);

        waitForStart();



        while (opModeIsActive()) {
            swerve.drive(gamepad1);

            idle();
        }
    }
}