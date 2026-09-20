package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Controls the intake motor. */
public class Intake {

    private static final double INTAKE_POWER = 1.0;

    private final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void runForward() {
        motor.setPower(INTAKE_POWER);
    }

    public void runReverse() {
        motor.setPower(-INTAKE_POWER);
    }

    public void stop() {
        motor.setPower(0.0);
    }

    public double getPower() {
        return motor.getPower();
    }
}
