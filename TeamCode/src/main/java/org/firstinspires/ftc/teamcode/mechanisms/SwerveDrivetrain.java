package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveDrivetrain {
    private static final double TRACK_WIDTH = 14.6;
    private static final double WHEEL_BASE = 11.125;
    private static final double R = Math.hypot(WHEEL_BASE, TRACK_WIDTH);

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final Servo[] steeringServos = new Servo[4];


    private IMU imu;


    public void init(HardwareMap hwMap) {
       driveMotors[0] = hwMap.get(DcMotorEx.class, "frontLeftDrive");
       driveMotors[1] = hwMap.get(DcMotorEx.class, "frontRightDrive");
       driveMotors[2] = hwMap.get(DcMotorEx.class, "backLeftDrive");
       driveMotors[3] = hwMap.get(DcMotorEx.class, "backRightDrive");


       steeringServos[0] = hwMap.get(Servo.class, "frontLeftSteer");
       steeringServos[1] = hwMap.get(Servo.class, "frontRightSteer");
       steeringServos[2] = hwMap.get(Servo.class, "backLeftSteer");
       steeringServos[3] = hwMap.get(Servo.class, "backRightSteer");

        IMU.Parameters IMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO Change to Logo Direction
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD // TODO Change to USB Direction
                )
        );

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(IMUparameters);
        imu.resetYaw();


    }

    public void drive(Gamepad gamepad, Telemetry telemetry) {
        // Reset heading when options or back button is pressed
        if (gamepad.options || gamepad.back) {
            imu.resetYaw();
        }

        double headingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double joystickX = gamepad.left_stick_x;
        double joystickY = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;

        telemetry.addData("Heading", headingRadians);
        telemetry.addLine();

        // Field Centric Conversion
        double cosHeading = Math.cos(headingRadians);
        double sinHeading = Math.sin(headingRadians);

        double fwd = joystickY * cosHeading + joystickX * sinHeading;
        double strafe = -joystickY * sinHeading + joystickX * cosHeading;

        // Swerve Drivetrain Math
        double a = strafe - turn * (WHEEL_BASE / R);
        double b = strafe + turn * (WHEEL_BASE / R);
        double c = fwd - turn * (TRACK_WIDTH / R);
        double d = fwd + turn * (TRACK_WIDTH / R);

        // Calculate Target Speeds for each module
        double flSpeed = Math.hypot(b, p(c)); // Front Left
        double frSpeed = Math.hypot(b, p(d)); // Front Right
        double blSpeed = Math.hypot(a, p(c)); // Back Left
        double brSpeed = Math.hypot(a, p(d)); // Back Right

        // Calculate Target Angles for each module
        double flAngle = Math.toDegrees(Math.atan2(b, c));
        double frAngle = Math.toDegrees(Math.atan2(b, d));
        double blAngle = Math.toDegrees(Math.atan2(a, c));
        double brAngle = Math.toDegrees(Math.atan2(a, d));

        // Normalize speeds so they never try to exceed 1.0 power scale
        double maxSpeed = Math.max(Math.max(flSpeed, frSpeed), Math.max(blSpeed, brSpeed));
        if (maxSpeed > 1.0) {
            flSpeed /= maxSpeed;
            frSpeed /= maxSpeed;
            blSpeed /= maxSpeed;
            brSpeed /= maxSpeed;
        }

        driveModule frontLeft = new driveModule(driveMotors[0], steeringServos[0]);
        driveModule frontRight = new driveModule(driveMotors[1], steeringServos[1]);
        driveModule backLeft = new driveModule(driveMotors[2], steeringServos[2]);
        driveModule backRight = new driveModule(driveMotors[3], steeringServos[3]);

//        This code runs the function in the driveModule class where the motor and servo position are printed in the terminal

        frontLeft.displayTelemetry(telemetry, flAngle, flSpeed, steeringServos[0], "Front Left");
        telemetry.addLine();
        frontRight.displayTelemetry(telemetry, frAngle, frSpeed, steeringServos[1], "Front Right");
        telemetry.addLine();
        backLeft.displayTelemetry(telemetry, blAngle, blSpeed, steeringServos[2], "Back Left");
        telemetry.addLine();
        backRight.displayTelemetry(telemetry, brAngle, brSpeed, steeringServos[3], "Back Right");
        telemetry.addLine();

        frontLeft.update(flSpeed, flAngle);
        frontRight.update(frSpeed, frAngle);
        backLeft.update(blSpeed, blAngle);
        backRight.update(brSpeed, brAngle);
    }

    public void setServoPosition(double servoPos) {
        for (Servo servo : steeringServos) {
            servo.setPosition(servoPos);
        }
    }


    // Quick helper method to safely handle internal naming math parameters
    private double p(double value) { return value; }
}
