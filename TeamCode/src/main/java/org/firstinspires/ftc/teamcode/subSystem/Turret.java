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
 * 行程（从正上方看）：起点 0°，只能顺时针转到终点 270°，再逆时针转回起点；
 * 起点和终点之间的 90° 不能经过，会扯到线。
 * Travel (viewed from above): from the start at 0° it may only turn clockwise to the end at 270°,
 * then counter-clockwise back; the 90° between end and start must never be crossed or it pulls the wires.
 *
 * 连续旋转模式下舵机没有位置反馈，所以用“动力 × 时间”估算角度；初始化前必须把转台摆到起点。
 * In continuous mode the servo has no position feedback, so the angle is estimated from power × time;
 * put the turret at the start before init.
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

    // 起点到终点的总行程，顺时针为正。 / Total travel from start to end, clockwise positive.
    private static final double TRAVEL_DEG = 270.0;

    // 全速从起点转到终点需要的秒数，必须实测：在 Turret Test 里按住 X 全速转一次，读屏幕上的 "Full-power sec"。
    // 暂定值按 110 RPM 舵机、132:51 齿轮比估算。
    // Seconds at full power from start to end; must be measured: in Turret Test hold X for one full-power run
    // and read "Full-power sec". The placeholder assumes a 110 RPM servo through the 132:51 gearing.
    private static final double FULL_POWER_TRAVEL_SECONDS = 1.06;

    // 离限位这么近时降速，减少惯性冲过限位。
    // Slow down this close to a limit to reduce overshoot from momentum.
    private static final double SLOW_ZONE_DEG = 30.0;
    private static final double SLOW_ZONE_MAX_POWER = 0.3;

    private final CRServo servo;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // 累计的“动力 × 秒”：全速转 1 秒加 1；除以 FULL_POWER_TRAVEL_SECONDS 就是行程比例。
    // Accumulated power × seconds: one second at full power adds 1; divided by FULL_POWER_TRAVEL_SECONDS
    // it is the fraction of travel.
    private double powerSeconds = 0.0;
    private double commandedPower = 0.0;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, DEVICE_NAME);
        servo.setDirection(DIRECTION);
        stop();
    }

    /**
     * 按比例旋转：正值顺时针，负值逆时针，范围 -1 到 1；在起点只能顺时针，在终点只能逆时针。
     * 每轮循环调用一次，角度估算依赖它。
     * Rotates proportionally: positive is clockwise, negative is counter-clockwise, range -1 to 1;
     * at the start it only turns clockwise and at the end only counter-clockwise.
     * Call once per loop; the angle estimate depends on it.
     */
    public void rotate(double speed) {
        updateEstimate(true);
        double power = Range.clip(speed, -1.0, 1.0) * MAX_POWER;
        double angle = getEstimatedAngle();
        if (power > 0.0) {
            if (angle >= TRAVEL_DEG) {
                power = 0.0;
            } else if (angle >= TRAVEL_DEG - SLOW_ZONE_DEG) {
                power = Math.min(power, SLOW_ZONE_MAX_POWER);
            }
        } else if (power < 0.0) {
            if (angle <= 0.0) {
                power = 0.0;
            } else if (angle <= SLOW_ZONE_DEG) {
                power = Math.max(power, -SLOW_ZONE_MAX_POWER);
            }
        }
        setPower(power);
    }

    /**
     * 不受限位地旋转，只用于标定和把转台救回行程内；操作时必须盯着线。
     * Rotates without limits, only for calibration and recovering the turret; watch the wires while using it.
     */
    public void rotateIgnoringLimits(double speed) {
        updateEstimate(false);
        setPower(Range.clip(speed, -1.0, 1.0) * MAX_POWER);
    }

    /** 停止旋转。 / Stops rotating. */
    public void stop() {
        updateEstimate(true);
        setPower(0.0);
    }

    /** 把当前位置当作起点（0°）。 / Treats the current position as the start (0°). */
    public void resetToStart() {
        powerSeconds = 0.0;
    }

    /** 估算的转台角度，起点 0°，顺时针为正。 / Estimated turret angle: start is 0°, clockwise positive. */
    public double getEstimatedAngle() {
        return powerSeconds / FULL_POWER_TRAVEL_SECONDS * TRAVEL_DEG;
    }

    /** 累计的全速秒数，标定 FULL_POWER_TRAVEL_SECONDS 用。 / Accumulated full-power seconds, for calibration. */
    public double getFullPowerSeconds() {
        return powerSeconds;
    }

    /** 是否停在起点或终点。 / Whether the estimate is at the start or the end. */
    public boolean isAtLimit() {
        double angle = getEstimatedAngle();
        return angle <= 0.0 || angle >= TRAVEL_DEG;
    }

    /** 返回当前设定动力，供 Telemetry 显示。 / Returns commanded power for Telemetry. */
    public double getPower() {
        return commandedPower;
    }

    private void updateEstimate(boolean clampToTravel) {
        // 用上一轮的动力乘以经过的时间累加。
        // Accumulate the previous loop's power times the elapsed time.
        double dt = loopTimer.seconds();
        loopTimer.reset();
        powerSeconds += commandedPower * dt;
        if (clampToTravel) {
            powerSeconds = Range.clip(powerSeconds, 0.0, FULL_POWER_TRAVEL_SECONDS);
        }
    }

    private void setPower(double power) {
        commandedPower = power;
        servo.setPower(power);
    }
}
