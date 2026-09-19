package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "test1 - Basic", group = "test1")
public class Test1TeleOP extends LinearOpMode {

    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    private DcMotorEx intake;

    // Intake 首次使用 80% 动力，减少链条和滚轴突然启动时的冲击。现在是100%
    private static final double INTAKE_POWER = 1.00;

    // 忽略摇杆中心附近的小偏差，防止机器人在没有操作时缓慢移动。
    // Ignore small stick errors near center so the robot does not creep when untouched.
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        // 这些名字必须和 Driver Station 中 test1 配置完全一致。
        // These names must exactly match the active test1 configuration on Driver Station.
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // 常见麦轮底盘需要反转左侧电机。实际方向会在轮子离地测试后确认。
        // A typical mecanum drivetrain reverses the left motors. Verify this with the wheels raised.
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopDrive();

        // Intake 位于 Expansion Hub Motor Port 0，第一版不使用 encoder。
        // The intake is on Expansion Hub Motor Port 0; this first version does not use its encoder.
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);

        telemetry.addLine("test1 initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // 这个底盘的实际电机方向需要直接使用摇杆 Y 值，推前时机器人才能向前。
            // This drivetrain uses the raw stick Y value so pushing forward moves the robot forward.
            double forward = applyDeadband(gamepad1.left_stick_y);

            // 反转两个水平轴以匹配实际底盘：左摇杆方向对应横移，右摇杆方向对应旋转。
            // Invert both horizontal axes to match the drivetrain's actual strafe and turn directions.
            double strafe = applyDeadband(-gamepad1.left_stick_x);
            double turn = applyDeadband(-gamepad1.right_stick_x);

            /*
             * 把前后、横移和旋转三个动作混合成四个轮子的动力。
             * Mix forward, strafe, and turn commands into four mecanum wheel powers.
             *
             * 分母用于按比例缩小所有轮子的动力，防止任何一个结果超过 1.0，
             * 同时保持机器人原本的移动方向。
             * The denominator scales every wheel together so no result exceeds 1.0
             * while preserving the requested movement direction.
             */
            double denominator = Math.max(
                    Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                    1.0
            );

            double flPower = (forward + strafe + turn) / denominator;
            double blPower = (forward - strafe + turn) / denominator;
            double frPower = (forward - strafe - turn) / denominator;
            double brPower = (forward + strafe - turn) / denominator;

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            /*
             * 只按住 A 时正转，只按住 B 时反转；松开或同时按下时停止。
             * Hold A for one direction and B for the opposite direction.
             * Releasing both buttons, or pressing both together, stops the intake.
             */
            double intakePower;
            if (gamepad1.a && !gamepad1.b) {
                intakePower = INTAKE_POWER;
            } else if (gamepad1.b && !gamepad1.a) {
                intakePower = -INTAKE_POWER;
            } else {
                intakePower = 0.0;
            }
            intake.setPower(intakePower);

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Intake", "%.2f", intakePower);
            telemetry.addData("fl / fr", "%.2f / %.2f", flPower, frPower);
            telemetry.addData("bl / br", "%.2f / %.2f", blPower, brPower);
            telemetry.update();
        }

        stopDrive();
        intake.setPower(0.0);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        fl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    private void stopDrive() {
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
    }
}
