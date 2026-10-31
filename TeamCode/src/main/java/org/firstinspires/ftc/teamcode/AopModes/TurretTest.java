package org.firstinspires.ftc.teamcode.AopModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystem.Turret;

/**
 * 只测试 Shooter 旋转舵机，只需要在 Driver Station 配置 "turret" 一个设备。
 * Tests only the shooter rotation servo; only the "turret" device needs to be configured on Driver Station.
 *
 * 任一手柄：RT 顺时针 | LT 逆时针 | BACK 当前位置设为起点 | 按住 X 不受限位（标定用）
 * Either gamepad: RT clockwise | LT counter-clockwise | BACK set current position as start
 * | hold X to ignore limits (calibration)
 *
 * 标定 FULL_POWER_TRAVEL_SECONDS：转台在起点按 BACK，按住 X 再把 RT 按到底，到终点立刻松开 RT，
 * 读屏幕上的 "Full-power sec"。
 * Calibrating FULL_POWER_TRAVEL_SECONDS: at the start press BACK, hold X and press RT fully,
 * release RT right at the end, and read "Full-power sec".
 */
@TeleOp(name = "Turret Test", group = "test1")
public class TurretTest extends LinearOpMode {

    // 忽略扳机的微小读数，防止松手时转台缓慢转动。
    // Ignore tiny trigger readings so the turret does not creep when untouched.
    private static final double TRIGGER_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);

        telemetry.addLine("Turret Test initialized");
        telemetry.addLine("Turn the turret to the START before init!");
        telemetry.addLine("RT: CW | LT: CCW | BACK: set start");
        telemetry.addLine("Hold X: ignore limits (watch the wires!)");
        telemetry.update();

        waitForStart();

        // 如果在开始前按下停止，就不要进入主循环。
        // Do not enter the main loop if Stop was requested before starting.
        if (isStopRequested()) {
            return;
        }

        // 最近一次按住 X 时的全速秒数，松开 X 后仍保留显示。
        // Full-power seconds from the last time X was held, kept on screen after X is released.
        double calibrationSeconds = 0.0;

        while (opModeIsActive()) {
            // 转台位置靠估算，漂了就转回起点再按 BACK。
            // The turret position is estimated; if it drifts, turn it back to the start and press BACK.
            if (gamepad1.back || gamepad2.back) {
                turret.resetToStart();
            }

            // 两个手柄都能用，取扳机按得更深的那个；RT 顺时针、LT 逆时针。
            // Either gamepad works, using whichever trigger is pressed deeper; RT clockwise, LT counter-clockwise.
            double rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            double leftTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            double clockwise = applyDeadband(rightTrigger - leftTrigger);
            boolean ignoreLimits = gamepad1.x || gamepad2.x;
            if (ignoreLimits) {
                turret.rotateIgnoringLimits(clockwise);
                calibrationSeconds = turret.getFullPowerSeconds();
            } else {
                turret.rotate(clockwise);
            }

            telemetry.addData("Limits", ignoreLimits ? "OFF (X held)" : "ON");
            telemetry.addData("Input", "%.2f", clockwise);
            telemetry.addData("Power", "%.2f", turret.getPower());
            telemetry.addData(
                    "Angle (est)",
                    "%.0f°%s",
                    turret.getEstimatedAngle(),
                    turret.isAtLimit() ? " | LIMIT" : ""
            );
            telemetry.addData("Full-power sec", "%.2f", calibrationSeconds);
            telemetry.update();
        }

        // OpMode 结束时停止旋转。
        // Stop rotating when the OpMode ends.
        turret.stop();
    }

    /**
     * 将死区范围内的输入变为零。
     * Converts input inside the deadband to zero.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < TRIGGER_DEADBAND ? 0.0 : value;
    }
}
