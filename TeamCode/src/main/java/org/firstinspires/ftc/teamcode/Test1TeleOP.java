package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

/**
 * 负责读取手柄输入，并协调底盘和 Intake 两个 subsystem。
 * Reads gamepad input and coordinates the drivetrain and intake subsystems.
 */
@TeleOp(name = "test1 - Basic", group = "test1")
public class Test1TeleOP extends LinearOpMode {

    private Drivetrain drivetrain;
    private Intake intake;

    // 忽略摇杆中心附近的小偏差，防止机器人在没有操作时缓慢移动。
    // Ignore small stick errors near center so the robot does not creep when untouched.
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        // 每个 subsystem 负责初始化和管理属于自己的硬件。
        // Each subsystem initializes and manages its own hardware.
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);

        // 告诉 Driver Station 所有硬件已完成初始化，可以开始运行。
        // Tell the Driver Station that all hardware is initialized and ready.
        telemetry.addLine("test1 initialized");
        telemetry.update();

        waitForStart();

        // 如果在开始前按下停止，就不要进入主循环。
        // Do not enter the main loop if Stop was requested before starting.
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // 这个底盘的实际电机方向需要直接使用摇杆 Y 值，推前时机器人才能向前。
            // This drivetrain uses the raw Y value so pushing the stick forward moves the robot forward.
            double forward = applyDeadband(gamepad1.left_stick_y);

            // 反转两个水平轴以匹配实际底盘：左摇杆方向对应横移，右摇杆方向对应旋转。
            // Invert both horizontal axes to match the robot's actual strafe and turn directions.
            double strafe = applyDeadband(-gamepad1.left_stick_x);
            double turn = applyDeadband(-gamepad1.right_stick_x);

            // TeleOp 只发出移动指令，具体的麦轮计算由 Drivetrain 完成。
            // TeleOp sends the command; Drivetrain performs the mecanum wheel calculations.
            drivetrain.drive(forward, strafe, turn);

            // 只按住 A 时正转，只按住 B 时反转；松开或同时按下时停止。
            // Hold only A to run forward and only B to reverse; release or hold both to stop.
            if (gamepad1.a && !gamepad1.b) {
                intake.runForward();
            } else if (gamepad1.b && !gamepad1.a) {
                intake.runReverse();
            } else {
                intake.stop();
            }

            // 显示输入值和实际输出动力，方便测试和排查问题。
            // Display input values and commanded motor powers for testing and troubleshooting.
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

        // OpMode 结束时明确停止所有机构，避免电机继续保持上一次的动力。
        // Explicitly stop every mechanism when the OpMode ends.
        drivetrain.stop();
        intake.stop();
    }

    /**
     * 将死区范围内的摇杆输入变为零。
     * Converts joystick input inside the deadband to zero.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }
}
