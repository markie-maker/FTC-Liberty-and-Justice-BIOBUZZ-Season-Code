package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.SwerveDrivetrain;

@TeleOp(name="Align Servos Teleop", group="Swerve Drivetrain Tests")
public class alignServosTeleop extends LinearOpMode {
    SwerveDrivetrain swerve = new SwerveDrivetrain();

    @Override
    public void runOpMode() {
        swerve.init(hardwareMap);
        telemetry.addLine("swerve initialization was a success!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            swerve.setServoPosition(0.5);
            idle();
        }
    }
}
