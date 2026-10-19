package org.firstinspires.ftc.teamcode.aopModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometrySystem.Pinpoint;

/**
 * 手推机器人检查里程计的测试模式，只读取传感器并显示数据，不控制电机。
 * Push the robot by hand to test odometry; this OpMode only reads sensors and displays data, with no motor control.
 * INIT 静止校准，READY 后 START；运行中静止按 A，可重新建立坐标原点。
 * Calibrate at rest during INIT, press START after READY, and press A while still to set a new origin.
 */
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {
    // reset 后至少等待 0.5 秒，避免立即把之前的 READY 当作校准完成。
    // Wait at least 0.5 seconds after reset to avoid accepting the previous READY status immediately.
    // 等待时间结束后仍需设备报告 READY，不能只靠计时判断。
    // The device must also report READY; elapsed time alone is not enough.
    private static final double RESET_SETTLE_SECONDS = 0.5;

    @Override
    public void runOpMode() {
        // 封装类负责硬件配置；计时器记录距上次清零过去了多久。
        // The wrapper configures the hardware; the timer tracks time since the last reset.
        Pinpoint pinpoint = new Pinpoint(hardwareMap);
        ElapsedTime resetTimer = new ElapsedTime();

        // 按 INIT 后会执行这里：当前的位置和朝向成为零点，校准期间不要移动车。
        // This runs after INIT: the current pose becomes zero; do not move during calibration.
        pinpoint.resetPosAndIMU();
        resetTimer.reset();

        // START 前持续刷新状态，便于观察校准过程；按 STOP 可直接退出等待。
        // Refresh status before START so calibration is visible; STOP exits this wait.
        while (!isStarted() && !isStopRequested()) {
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            telemetry.addLine("INIT: Keep robot still");
            telemetry.addData("Pinpoint", status);
            if (isReady(status, resetTimer)) {
                telemetry.addLine("READY: Press START");
            } else {
                telemetry.addLine("Waiting for READY; readings are invalid");
            }
            telemetry.update();
            idle();
        }

        // 如果在 INIT 阶段停止，就不进入下面的运行循环。
        // If stopped during INIT, skip the running loop below.
        if (isStopRequested()) {
            return;
        }

        // 记录 START 时的 A 状态；此前已经按住的 A 不算一次新按键。
        // Remember A at START; an already-held button does not count as a new press.
        boolean previousA = gamepad1.a;
        while (opModeIsActive()) {
            // 上升沿 = 上一轮没按、本轮按下；所以长按 A 只触发一次。
            // A rising edge means released last loop and pressed now, so holding A triggers only once.
            boolean aPressed = gamepad1.a;
            boolean aRisingEdge = aPressed && !previousA;
            previousA = aPressed;

            // 先更新，再读状态和位置，确保这一轮使用新数据。
            // Update first, then read status and pose so this loop uses fresh data.
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            if (aRisingEdge && isReady(status, resetTimer)) {
                // 操作者必须先停稳再按 A；程序不会自动判断车是否静止。
                // The operator must stop the robot before pressing A; this code does not detect stillness.
                // 校准或故障期间忽略 A，需要恢复 READY 后松开并重新按下。
                // A is ignored during calibration or a fault; release and press again after READY.
                pinpoint.resetPosAndIMU();
                resetTimer.reset();
            }

            telemetry.addData("Pinpoint", status);
            telemetry.addLine("A: Reset pose + IMU only while STILL");
            if (isReady(status, resetTimer)) {
                // 位置单位为毫米，航向单位为度。Pose2D 会归一化角度，转一圈可回到 0°。
                // Position is in millimeters and heading in degrees; Pose2D normalizes heading, so a full turn can return to 0°.
                Pose2D pose = pinpoint.getPosition();
                telemetry.addData("X forward (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                telemetry.addData("Y left (mm)", "%.1f", pose.getY(DistanceUnit.MM));
                telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            } else {
                // 未就绪时不显示位置数值，避免把校准中的读数当作有效测量。
                // Hide pose values while not ready so calibration readings are not mistaken for valid measurements.
                telemetry.addLine("WAIT / FAULT: Pose invalid until READY");
                telemetry.addLine("Keep robot still; check pods and wiring if fault persists");
            }
            telemetry.update();
            idle();
        }
    }

    /**
     * 同时满足设备 READY 和清零后的最短等待时间，才允许显示位置或再次清零。
     * Allow pose display or another reset only when the device is READY and the minimum reset wait has passed.
     */
    private boolean isReady(GoBildaPinpointDriver.DeviceStatus status, ElapsedTime resetTimer) {
        return status == GoBildaPinpointDriver.DeviceStatus.READY
                && resetTimer.seconds() >= RESET_SETTLE_SECONDS;
    }
}
