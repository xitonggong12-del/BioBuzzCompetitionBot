package org.firstinspires.ftc.teamcode.aopModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystem.Shooter;

/**
 * 只测试 Shooter：检查转向和 encoder，并以全动力向外发射或反转。
 * Tests only the shooter: checks direction and encoder, and runs full power outward or in reverse.
 */
@TeleOp(name = "Shooter Test", group = "test1")
public class ShooterTest extends LinearOpMode {

    // 低速开环动力，用来安全地确认转向和 encoder 读数。
    // Low open-loop power for safely checking direction and encoder readings.
    private static final double CHECK_POWER = 0.3;

    // 发射和反转都用全动力开环。
    // Both launch and reverse use full open-loop power.
    private static final double FULL_POWER = 1.0;

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        telemetry.addLine("Shooter Test initialized");
        telemetry.addLine("LB: launch 100% | RB: reverse 100% | A: check 30%");
        telemetry.update();

        waitForStart();

        // 如果在开始前按下停止，就不要进入主循环。
        // Do not enter the main loop if Stop was requested before starting.
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            // 一次只执行一个按键；没有按或同时按多个时滑行停止。
            // Run only one button at a time; with none or several held, coast to a stop.
            boolean checkHeld = gamepad1.a;
            boolean launchHeld = gamepad1.left_bumper;
            boolean reverseHeld = gamepad1.right_bumper;
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

            // 向外发射时实际速度应为正；如果为负，说明 encoder 方向和电机方向不一致。
            // Velocity should be positive when launching; if negative, the encoder and motor directions disagree.
            double velocity = shooter.getVelocity();
            telemetry.addData("Mode", mode);
            telemetry.addData("Actual", "%.0f tps / %.0f RPM", velocity, Shooter.toRpm(velocity));
            telemetry.addData("Power", "%.2f", shooter.getPower());
            telemetry.addData("Current", "%.2f A", shooter.getCurrentAmps());
            telemetry.update();
        }

        // OpMode 结束时让飞轮滑行停下。
        // Let the flywheel coast down when the OpMode ends.
        shooter.stop();
    }
}
