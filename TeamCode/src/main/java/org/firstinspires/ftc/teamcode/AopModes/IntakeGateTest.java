package org.firstinspires.ftc.teamcode.AopModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subSystem.IntakeGate;

/**
 * 找 Intake 限位舵机的三个位置（STOWED / DROP / GAP），把读数填进 IntakeGate。
 * Finds the three intake stop servo positions (STOWED / DROP / GAP) to fill into IntakeGate.
 *
 * 任一手柄：十字键上/下 舵机 +0.01/-0.01
 * Either gamepad: D-pad up/down servo +0.01/-0.01
 *
 * 方向和 IntakeGate 一样，所以屏幕上的数值可以直接填进去。
 * The direction matches IntakeGate, so the number on screen can be copied straight in.
 */
@TeleOp(name = "Intake Gate Test", group = "test1")
public class IntakeGateTest extends LinearOpMode {

    // 每按一下改变的位置。 / Position change per press.
    private static final double STEP = 0.01;

    // 起始数值；舵机在第一次按键前不会动，避免一开始就猛转。
    // Starting value; the servo does not move until the first press, so nothing jumps at start.
    private static final double START_POSITION = 0.5;

    @Override
    public void runOpMode() {
        Servo servo = hardwareMap.get(Servo.class, IntakeGate.DEVICE_NAME);
        servo.setDirection(IntakeGate.DIRECTION);

        telemetry.addLine("Intake Gate Test initialized");
        telemetry.addLine("D-pad up/down: +0.01 / -0.01");
        telemetry.update();

        waitForStart();

        // 如果在开始前按下停止，就不要进入主循环。
        // Do not enter the main loop if Stop was requested before starting.
        if (isStopRequested()) {
            return;
        }

        double position = START_POSITION;
        boolean moved = false;

        // 记录上一轮的按键，只在按下那一下改变数值，长按不会连续跳。
        // Remember last loop's buttons so each press changes the value once; holding does not repeat.
        boolean previousUp = false;
        boolean previousDown = false;

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean down = gamepad1.dpad_down || gamepad2.dpad_down;

            if (up && !previousUp) {
                position = Range.clip(position + STEP, 0.0, 1.0);
                moved = true;
            } else if (down && !previousDown) {
                position = Range.clip(position - STEP, 0.0, 1.0);
                moved = true;
            }
            previousUp = up;
            previousDown = down;

            if (moved) {
                servo.setPosition(position);
            }

            telemetry.addData("Position", "%.2f%s", position, moved ? "" : " (not moved yet)");
            telemetry.update();
        }
    }
}
