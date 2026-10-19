package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class driveModule {
    final double maxServoRotation = 180.0;
    private final Servo servo;
    private final DcMotorEx motor;


    public driveModule(DcMotorEx motor, Servo servo) {
        this.motor = motor;
        this.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.servo = servo;
    }

    public void update(double motorSpeed, double targetAngle) {
        // -90 = left, 0 = middle, 90 = right
        double turnMax =  maxServoRotation/2; // 90
        double turnMin = -maxServoRotation/2; // -90


        // reverse motor if target angle is too high/low because the servo can only go 180 deg
        if (targetAngle > turnMax) {
            targetAngle -= maxServoRotation;
            motorSpeed = -motorSpeed;
        } else if (targetAngle < turnMin) {
            targetAngle += maxServoRotation;
            motorSpeed = -motorSpeed;
        }

        double targetPositon = normalize(targetAngle, turnMin, turnMax, 0.0,1.0);

        if (targetPositon < 0.0) {
            targetPositon = 0.0;
        }

        if (targetPositon > 1.0) {
            targetPositon = 1.0;
        }

        servo.setPosition(targetPositon);

        // use 2000 instead of 2500 to restrain from going full power
        final double motorTicksPerSecond = 2000.0;

        // ticks per second
        double targetVelocityTicks = motorSpeed * motorTicksPerSecond;
        motor.setVelocity(targetVelocityTicks);
    }

    public static double normalize(double num, double min, double max, double newMin, double newMax) {
        // function I found online

        if (max == min) {
            return newMin;
        }

        return ((num - min) / (max - min)) * (newMax - newMin) + newMin;
    }
}
