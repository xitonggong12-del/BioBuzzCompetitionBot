package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * 管理飞轮电机：开环低速测试、encoder 速度闭环、转速就绪判断和滑行停车。
 * Manages the flywheel motor: open-loop test power, encoder velocity control, ready check, and coast stop.
 *
 * 速度单位统一为 ticks/秒（encoder 原始单位），RPM 只用于显示。
 * Velocity is always in ticks per second (raw encoder units); RPM is for display only.
 */
public class Shooter {

    // 名称 "shot" 必须和 Driver Station 中的机器人配置一致（Control Hub 电机 0 号口）。
    // The name "shot" must match the robot configuration on Driver Station (Control Hub motor port 0).
    private static final String DEVICE_NAME = "shot";

    // 实测 FORWARD 的负动力才是向外发射，所以设为 REVERSE，让正指令等于发射方向。
    // On the robot, negative power with FORWARD launched outward, so REVERSE makes a positive command launch.
    private static final DcMotor.Direction DIRECTION = DcMotor.Direction.REVERSE;

    // 电机输出轴每转 encoder 计数，只影响 RPM 显示，不影响控制。
    // Encoder counts per output-shaft revolution; affects only the RPM display, not control.
    // goBILDA 5203 6000 RPM 是 1:1 无减速，每转 28；最高约 2800 ticks/秒。
    // The goBILDA 5203 6000 RPM is 1:1 with no gearbox, reading 28 per revolution; top speed is about 2800 ticks/s.
    private static final double TICKS_PER_REV = 28.0;

    // 实际速度与目标相差不超过该值才算到速；数值需实测后调整。
    // The flywheel counts as at speed only within this error; tune after testing.
    private static final double READY_TOLERANCE_TPS = 50.0;

    // 必须连续到速这么久才算 READY，避免转速刚好经过目标时误判。
    // Must stay at speed this long before READY, so passing through the target does not count.
    private static final double READY_HOLD_SECONDS = 0.2;

    private final DcMotorEx motor;
    private final ElapsedTime atSpeedTimer = new ElapsedTime();

    private DcMotor.RunMode currentMode;
    private double targetVelocityTps = 0.0;
    private boolean velocityControlActive = false;
    private boolean wasAtSpeed = false;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, DEVICE_NAME);
        motor.setDirection(DIRECTION);

        // 飞轮惯量大，BRAKE 急停会冲击齿轮和链条，所以停止时让它自由滑行。
        // The flywheel has high inertia; BRAKE would shock the chain and gears, so let it coast.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /**
     * 以 encoder 速度闭环维持目标转速；encoder 未接时电机会全速运转，先用开环测试确认读数。
     * Holds the target speed with encoder velocity control; without an encoder the motor runs at full power,
     * so confirm readings with the open-loop test first.
     */
    public void setTargetVelocity(double ticksPerSecond) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetVelocityTps = ticksPerSecond;
        velocityControlActive = true;
        motor.setVelocity(ticksPerSecond);
    }

    /**
     * 不使用 encoder，直接给固定动力；用于检查转向和 encoder 读数。
     * Applies fixed power without the encoder; used to check direction and encoder readings.
     */
    public void setOpenLoopPower(double power) {
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetVelocityTps = 0.0;
        velocityControlActive = false;
        motor.setPower(power);
    }

    /** 切回开环并将动力归零，飞轮滑行停下。 / Returns to open loop at zero power so the flywheel coasts down. */
    public void stop() {
        setOpenLoopPower(0.0);
    }

    /**
     * 每轮循环调用一次，更新连续到速计时；isReady() 依赖它。
     * Call once per loop to update the at-speed timer that isReady() depends on.
     */
    public void update() {
        boolean atSpeed = isAtSpeed();
        if (atSpeed && !wasAtSpeed) {
            atSpeedTimer.reset();
        }
        wasAtSpeed = atSpeed;
    }

    /** 当前读数在目标误差范围内。 / The current reading is within tolerance of the target. */
    public boolean isAtSpeed() {
        return velocityControlActive
                && targetVelocityTps > 0.0
                && Math.abs(getVelocity() - targetVelocityTps) <= READY_TOLERANCE_TPS;
    }

    /** 连续到速达到保持时间，可以送球。 / At speed for the hold time, so it is safe to feed. */
    public boolean isReady() {
        return wasAtSpeed && atSpeedTimer.seconds() >= READY_HOLD_SECONDS;
    }

    public boolean isVelocityControlActive() {
        return velocityControlActive;
    }

    public double getTargetVelocity() {
        return targetVelocityTps;
    }

    /** 实测速度，单位 ticks/秒。 / Measured velocity in ticks per second. */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** ticks/秒换算为 RPM，仅供显示。 / Converts ticks per second to RPM for display only. */
    public static double toRpm(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / TICKS_PER_REV;
    }

    /** 返回当前设定动力，供 Telemetry 显示。 / Returns commanded power for Telemetry. */
    public double getPower() {
        return motor.getPower();
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        // 只在模式变化时写入，避免每轮循环重复发送硬件指令。
        // Write only when the mode changes to avoid resending the hardware command every loop.
        if (mode != currentMode) {
            motor.setMode(mode);
            currentMode = mode;
        }
    }
}
