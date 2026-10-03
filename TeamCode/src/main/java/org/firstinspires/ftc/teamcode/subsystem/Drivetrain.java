package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 管理四电机麦克纳姆轮底盘的硬件和移动计算。
 * Manages the hardware and movement calculations for the four-motor mecanum drivetrain.
 */
public class Drivetrain {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public Drivetrain(HardwareMap hardwareMap) {
        // 硬件名称必须和 Driver Station 中启用的机器人配置完全一致。
        // Hardware names must exactly match the active robot configuration on Driver Station.
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        // 左侧电机安装方向相反，因此需要在软件中反转。
        // The left motors are mounted in the opposite direction, so reverse them in software.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE 让摇杆松开后更快停车；当前驾驶不使用 encoder 闭环控制。
        // BRAKE stops the robot faster when sticks are released; driving currently uses no encoder loop.
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /**
     * 将前后、横移和旋转指令混合为四个轮子的动力。
     * Mixes forward, strafe, and turn commands into four wheel powers.
     */
    public void drive(double forward, double strafe, double turn) {
        // 按比例缩小所有轮子的动力，保证最大值不超过 1.0，同时保持移动方向。
        // Scale all wheel powers together so none exceeds 1.0 while preserving direction.
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );

        // 麦轮混合公式分别计算左前、左后、右前和右后轮的动力。
        // Mecanum mixing calculates power for the front-left, back-left, front-right, and back-right wheels.
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        frontRight.setPower((forward - strafe - turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }

    /** 停止全部底盘电机。 / Stops all drivetrain motors. */
    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    // 以下 getter 让 TeleOp 能显示电机动力，但不会直接控制底盘硬件。
    // These getters let TeleOp display motor powers without directly controlling drivetrain hardware.
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
        // 对四个底盘电机应用同一个停车行为。
        // Apply the same zero-power behavior to all four drivetrain motors.
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        // 对四个底盘电机应用同一个运行模式。
        // Apply the same run mode to all four drivetrain motors.
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}
