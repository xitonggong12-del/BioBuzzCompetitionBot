package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Controls the four-motor mecanum drivetrain. */
public class Drivetrain {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public Drivetrain(HardwareMap hardwareMap) {
        // Hardware names must match the active Driver Station configuration.
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /** Mixes forward, strafe, and turn commands into normalized wheel powers. */
    public void drive(double forward, double strafe, double turn) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );

        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        frontRight.setPower((forward - strafe - turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }

    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public double getFrontLeftPower() {
        return frontLeft.getPower();
    }

    public double getFrontRightPower() {
        return frontRight.getPower();
    }

    public double getBackLeftPower() {
        return backLeft.getPower();
    }

    public double getBackRightPower() {
        return backRight.getPower();
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}
