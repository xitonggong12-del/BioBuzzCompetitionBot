package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * 管理 Shooter 的旋转舵机（goBILDA Dual Mode Servo，切换为 360° 连续旋转模式）。
 * Manages the shooter rotation servo (goBILDA Dual Mode Servo, set to 360° continuous rotation mode).
 *
 * 连续旋转模式下舵机没有位置反馈，所以用“动力 × 时间”粗略估算转台角度，并在 ±135° 处停住。
 * In continuous mode the servo has no position feedback, so the turret angle is roughly estimated
 * from power × time, and rotation stops at ±135°.
 *
 * 估算会随时间漂移：初始化前必须把转台摆到行程正中间（估算角度 0°）。
 * The estimate drifts over time: put the turret at the middle of its travel (estimated 0°) before init.
 */
public class Turret {

    // 名称 "turret" 必须和 Driver Station 中的机器人配置一致，类型选 Continuous Rotation Servo。
    // The name "turret" must match the robot configuration on Driver Station, configured as a Continuous Rotation Servo.
    private static final String DEVICE_NAME = "turret";

    // 方向需实测：如果正指令不是顺时针（从上往下看），改成 REVERSE。
    // Verify on the robot: if a positive command is not clockwise (viewed from above), change to REVERSE.
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    // 旋转最大动力；转得太快不好瞄准时调小。
    // Maximum rotation power; lower it if the shooter turns too fast to aim.
    private static final double MAX_POWER = 1.0;

    // 齿轮比：转台齿圈 132 齿 / 舵机齿轮 51 齿，舵机转 2.59 圈转台转 1 圈。
    // Gear ratio: 132-tooth turret ring / 51-tooth servo pinion; 2.59 servo turns per turret turn.
    private static final double GEAR_RATIO = 132.0 / 51.0;

    // 舵机全速时的转速（RPM），暂定值，需实测：全速转 1 秒，量转台转了多少度，再反算。
    // Servo speed at full power (RPM), a placeholder to measure: run full power for 1 second,
    // measure how far the turret turned, and work back from that.
    private static final double SERVO_MAX_RPM = 110.0;

    // 转台全速时每秒转过的角度。 / Turret degrees per second at full power.
    private static final double TURRET_MAX_DEG_PER_SEC = SERVO_MAX_RPM * 360.0 / 60.0 / GEAR_RATIO;

    // 总行程 270°，从中间开始两边各 135°；还不知道限位在哪边，所以先对称处理。
    // 270° of total travel, 135° each way from center; the hard stops are not mapped yet, so treat it symmetrically.
    private static final double ANGLE_LIMIT_DEG = 135.0;

    private final CRServo servo;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double estimatedAngleDeg = 0.0;
    private double commandedPower = 0.0;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, DEVICE_NAME);
        servo.setDirection(DIRECTION);
        stop();
    }

    /**
     * 按比例旋转：正值顺时针，负值逆时针，范围 -1 到 1；到达估算限位后只允许往回转。
     * 每轮循环调用一次，角度估算依赖它。
     * Rotates proportionally: positive is clockwise, negative is counter-clockwise, range -1 to 1;
     * at an estimated limit it only turns back. Call once per loop; the angle estimate depends on it.
     */
    public void rotate(double speed) {
        updateEstimate();
        double power = Range.clip(speed, -1.0, 1.0) * MAX_POWER;
        if ((power > 0.0 && estimatedAngleDeg >= ANGLE_LIMIT_DEG)
                || (power < 0.0 && estimatedAngleDeg <= -ANGLE_LIMIT_DEG)) {
            power = 0.0;
        }
        setPower(power);
    }

    /** 停止旋转。 / Stops rotating. */
    public void stop() {
        updateEstimate();
        setPower(0.0);
    }

    /** 把当前位置当作行程中间（0°）。 / Treats the current position as the middle of travel (0°). */
    public void resetAngle() {
        estimatedAngleDeg = 0.0;
    }

    /** 估算的转台角度，顺时针为正。 / Estimated turret angle, clockwise positive. */
    public double getEstimatedAngle() {
        return estimatedAngleDeg;
    }

    /** 是否停在估算限位上。 / Whether the estimate is at a travel limit. */
    public boolean isAtLimit() {
        return Math.abs(estimatedAngleDeg) >= ANGLE_LIMIT_DEG;
    }

    /** 返回当前设定动力，供 Telemetry 显示。 / Returns commanded power for Telemetry. */
    public double getPower() {
        return commandedPower;
    }

    private void updateEstimate() {
        // 用上一轮的动力乘以经过的时间累加角度。
        // Accumulate angle from the previous loop's power times the elapsed time.
        double dt = loopTimer.seconds();
        loopTimer.reset();
        estimatedAngleDeg += commandedPower * TURRET_MAX_DEG_PER_SEC * dt;
        estimatedAngleDeg = Range.clip(estimatedAngleDeg, -ANGLE_LIMIT_DEG, ANGLE_LIMIT_DEG);
    }

    private void setPower(double power) {
        commandedPower = power;
        servo.setPower(power);
    }
}
