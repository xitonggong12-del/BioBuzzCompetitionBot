package org.firstinspires.ftc.teamcode.aopModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subSystem.Intake;
import org.firstinspires.ftc.teamcode.subSystem.Shooter;
import org.firstinspires.ftc.teamcode.subSystem.Turret;

/**
 * 测试 Shooter：gamepad1 是 driver 只负责驾驶，gamepad2 负责发射、Intake 和 Shooter 旋转。
 * Tests the shooter: gamepad1 is the driver and only drives; gamepad2 runs shooting, intake, and shooter rotation.
 *
 * gamepad2：LB 发射 | RB Intake 向里 | LT 逆时针 | RT 顺时针 | A 飞轮 30% | B 飞轮反转 | BACK 转台角度归零
 * gamepad2: LB shoot | RB intake in | LT counter-clockwise | RT clockwise | A flywheel 30% | B flywheel reverse
 * | BACK zero turret angle
 */
@TeleOp(name = "Shooter Test", group = "test1")
public class ShooterTest extends LinearOpMode {

    // 低速开环动力，用来安全地确认转向和 encoder 读数。
    // Low open-loop power for safely checking direction and encoder readings.
    private static final double CHECK_POWER = 0.3;

    // 发射和反转都用全动力开环。
    // Both launch and reverse use full open-loop power.
    private static final double FULL_POWER = 1.0;

    // 忽略摇杆和扳机的微小读数，防止松手时机构缓慢移动。
    // Ignore tiny stick and trigger readings so nothing creeps when untouched.
    private static final double JOYSTICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        telemetry.addLine("Shooter Test initialized");
        telemetry.addLine("Center the turret before init!");
        telemetry.addLine("GP1: drive");
        telemetry.addLine("GP2: LB shoot | RB intake | LT CCW | RT CW");
        telemetry.addLine("GP2: A check 30% | B reverse | BACK zero turret");
        telemetry.update();

        waitForStart();

        // 如果在开始前按下停止，就不要进入主循环。
        // Do not enter the main loop if Stop was requested before starting.
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // gamepad1：和 test1 相同的驾驶方式。
            // gamepad1: same driving as test1.
            double forward = applyDeadband(-gamepad1.left_stick_y);
            double strafe = applyDeadband(gamepad1.left_stick_x);
            double turn = applyDeadband(gamepad1.right_stick_x);
            drivetrain.drive(forward, strafe, turn);

            // 飞轮一次只执行一个按键；没有按或同时按多个时滑行停止。
            // Flywheel runs one button at a time; with none or several held, coast to a stop.
            boolean launchHeld = gamepad2.left_bumper;
            boolean checkHeld = gamepad2.a;
            boolean reverseHeld = gamepad2.b;
            String mode;
            if (launchHeld && !reverseHeld && !checkHeld) {
                shooter.setOpenLoopPower(FULL_POWER);
                mode = "LAUNCH 100%";
            } else if (reverseHeld && !launchHeld && !checkHeld) {
                shooter.setOpenLoopPower(-FULL_POWER);
                mode = "REVERSE 100%";
            } else if (checkHeld && !launchHeld && !reverseHeld) {
                shooter.setOpenLoopPower(CHECK_POWER);
                mode = "CHECK 30%";
            } else {
                shooter.stop();
                mode = "STOP (coast)";
            }

            // 按住 RB 时 Intake 向里转，松开停止。
            // Hold RB to run the intake inward; release to stop.
            if (gamepad2.right_bumper) {
                intake.runForward();
            } else {
                intake.stop();
            }

            // 转台位置靠估算，漂了就手动摆回中间再按 BACK 归零。
            // The turret position is estimated; if it drifts, move it back to center by hand and press BACK.
            if (gamepad2.back) {
                turret.resetAngle();
            }

            // RT 顺时针、LT 逆时针，按得越深转得越快；两个同时按会互相抵消。
            // RT turns clockwise and LT counter-clockwise, faster the deeper they are pressed;
            // pressing both cancels out.
            double clockwise = applyDeadband(gamepad2.right_trigger - gamepad2.left_trigger);
            turret.rotate(clockwise);

            // 向外发射时实际速度应为正；如果为负，说明 encoder 方向和电机方向不一致。
            // Velocity should be positive when launching; if negative, the encoder and motor directions disagree.
            double velocity = shooter.getVelocity();
            telemetry.addData("Mode", mode);
            telemetry.addData("Actual", "%.0f tps / %.0f RPM", velocity, Shooter.toRpm(velocity));
            telemetry.addData("Power", "%.2f", shooter.getPower());
            telemetry.addData("Current", "%.2f A", shooter.getCurrentAmps());
            telemetry.addData("Intake", "%.2f", intake.getPower());
            telemetry.addData(
                    "Turret",
                    "%.0f° (est) | power %.2f%s",
                    turret.getEstimatedAngle(),
                    turret.getPower(),
                    turret.isAtLimit() ? " | LIMIT" : ""
            );
            telemetry.update();
        }

        // OpMode 结束时停止所有机构，飞轮滑行停下。
        // Stop every mechanism when the OpMode ends; the flywheel coasts down.
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        turret.stop();
    }

    /**
     * 将死区范围内的输入变为零。
     * Converts input inside the deadband to zero.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }
}
