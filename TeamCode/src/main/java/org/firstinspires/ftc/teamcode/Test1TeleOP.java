package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(name = "test1 - Basic", group = "test1")
public class Test1TeleOP extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;

    // 忽略摇杆中心附近的小偏差，防止机器人在没有操作时缓慢移动。
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addLine("test1 initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // 这个底盘的实际电机方向需要直接使用摇杆 Y 值，推前时机器人才能向前。
            double forward = applyDeadband(gamepad1.left_stick_y);

            // 反转两个水平轴以匹配实际底盘：左摇杆方向对应横移，右摇杆方向对应旋转。
            double strafe = applyDeadband(-gamepad1.left_stick_x);
            double turn = applyDeadband(-gamepad1.right_stick_x);

            drivetrain.drive(forward, strafe, turn);

            // 只按住 A 时正转，只按住 B 时反转；松开或同时按下时停止。
            if (gamepad1.a && !gamepad1.b) {
                intake.runForward();
            } else if (gamepad1.b && !gamepad1.a) {
                intake.runReverse();
            } else {
                intake.stop();
            }

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Intake", "%.2f", intake.getPower());
            telemetry.addData(
                    "fl / fr",
                    "%.2f / %.2f",
                    drivetrain.getFrontLeftPower(),
                    drivetrain.getFrontRightPower()
            );
            telemetry.addData(
                    "bl / br",
                    "%.2f / %.2f",
                    drivetrain.getBackLeftPower(),
                    drivetrain.getBackRightPower()
            );
            telemetry.update();
        }

        drivetrain.stop();
        intake.stop();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }
}
