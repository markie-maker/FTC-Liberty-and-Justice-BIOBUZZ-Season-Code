package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class driveModule {
    private final Servo servo;
    private final DcMotorEx motor;


    public driveModule(DcMotorEx motor, Servo servo) {
        this.motor = motor;
        if (this.motor != null) {
            this.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        this.servo = servo;
//      just know it allows the axon servo to go the full 180deg range instead of 135-140deg
        if (this.servo instanceof ServoImplEx) {
            ((ServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    public void update(double motorSpeed, double targetAngle) {
        // dead zone to avoid noise
        if (Math.abs(motorSpeed) < 0.05) {
            if (motor != null) {
                motor.setPower(0);
            }
            return;
        }

        while (targetAngle > 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        double turnMax =  90.0; // Right
        double turnMin = -90.0; // Left

       // keep angle from going too high or low
        if (targetAngle > turnMax) {
            targetAngle -= 180;
            motorSpeed = -motorSpeed;
        } else if (targetAngle < turnMin) {
            targetAngle += 180;
            motorSpeed = -motorSpeed;
        }

        double targetPosition = normalize(targetAngle, turnMin, turnMax, 0.0, 1.0);
        targetPosition = Math.max(0.0, Math.min(1.0, targetPosition));

        // get firm values instead of numbers like 0.9873424
        if (Math.abs(targetPosition - 1.0) < 0.03) {
            targetPosition = 1.0;
        } else if (Math.abs(targetPosition - 0.0) < 0.03) {
            targetPosition = 0.0;
        } else if (Math.abs(targetPosition - 0.5) < 0.03) {
            targetPosition = 0.5;
        }

        if (servo != null) {
            servo.setPosition(targetPosition);
        }

        if (motor != null) {
            // use 2000 instead of 2500 to restrain from going full power
            final double motorTicksPerSecond = 2000.0;

            // ticks per second
            double targetVelocityTicks = motorSpeed * motorTicksPerSecond;
            motor.setVelocity(targetVelocityTicks);
        }
    }

    public void displayTelemetry(Telemetry telemetry, double angle, double speed, Servo servo, String moduleName) {
        if (telemetry == null) return;
        telemetry.addData(moduleName + " Angle", angle);
        telemetry.addData(moduleName + " Speed", speed);
        telemetry.addData(moduleName + " Servo Pos", servo != null ? servo.getPosition() : "NOT CONNECTED");
    }

    public static double normalize(double num, double min, double max, double newMin, double newMax) {
        if (max == min) {
            return newMin;
        }

        return ((num - min) / (max - min)) * (newMax - newMin) + newMin;
    }
}
