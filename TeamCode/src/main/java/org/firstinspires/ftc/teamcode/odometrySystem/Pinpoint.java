package org.firstinspires.ftc.teamcode.odometrySystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 负责 Pinpoint 的硬件配置、位置读取和清零，供不同 OpMode 重复使用。
 * Handles Pinpoint setup, pose readings, and resets for reuse by different OpModes.
 *
 * 追踪点是 13 × 13 英寸固定车架的几何中心，不计 intake 和车轮。
 * The tracking point is the geometric center of the 13 × 13 inch frame, excluding the intake and wheels.
 */
public class Pinpoint {
    // 必须与 Driver Station 硬件配置中的设备名称完全一致。
    // Must exactly match the device name in the Driver Station hardware configuration.
    private static final String DEVICE_NAME = "pinpoint";

    // 方向以车正常落地、从车尾看向车头为准。
    // Directions assume the robot is upright, viewed from the rear toward the front.
    // X pod 测量前后运动，但它的 offset 是横向安装距离：左正、右负。
    // The X pod measures forward/backward motion; its offset is sideways: left positive, right negative.
    // Y pod 测量左右运动，但它的 offset 是纵向安装距离：前正、后负。
    // The Y pod measures sideways motion; its offset is longitudinal: front positive, rear negative.
    // 这些固定测量值用于旋转补偿，不是启动位置；清零时也不改变它们。
    // These fixed measurements compensate for rotation; they are not the starting pose and survive resets.
    private static final double X_POD_OFFSET_MM = 101.6;   // 左侧 4 in / 4 in left of center
    private static final double Y_POD_OFFSET_MM = -146.05; // 后方 5.75 in / 5.75 in behind center

    // 初始均为 FORWARD，实际方向需要手推测试：向前 X 增加，向左 Y 增加。
    // Start with FORWARD for both; verify by pushing: forward increases X, left increases Y.
    // 若某一路方向相反，只将对应常量改为 REVERSED，不要用 offset 的正负来修正。
    // If one pod counts backward, change its constant to REVERSED; do not change the offset sign.
    private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private final GoBildaPinpointDriver device;

    /**
     * 按名称取得 SDK 内置驱动，并写入安装参数；调用方随后在静止时校准。
     * Gets the built-in SDK driver by name and applies mounting settings; the caller then calibrates at rest.
     */
    public Pinpoint(HardwareMap hardwareMap) {
        device = hardwareMap.get(GoBildaPinpointDriver.class, DEVICE_NAME);
        device.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        // Swingarm 预设把编码器计数换算成距离，需与实际使用的 pod 型号一致。
        // The Swingarm preset converts encoder counts to distance and must match the installed pods.
        device.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        device.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);
    }

    /**
     * 将当前位置与朝向设为 (0, 0, 0)，并重新校准 IMU。
     * Sets the current position and heading to (0, 0, 0) and recalibrates the IMU.
     * 机器人必须保持静止，调用后继续 update() 并等待 READY，再使用读数。
     * Keep the robot still; keep calling update() and wait for READY before using the readings.
     */
    public void resetPosAndIMU() {
        device.resetPosAndIMU();
    }

    /**
     * 每轮循环先调用一次，从设备获取新数据，再读取位置和状态。
     * Call once at the start of each loop to fetch fresh data before reading pose and status.
     */
    public void update() {
        device.update();
    }

    /**
     * 返回位置和航向；调用方用 DistanceUnit 和 AngleUnit 选择输出单位。
     * Returns position and heading; the caller selects units with DistanceUnit and AngleUnit.
     * 清零时的朝向定义 +X，左侧定义 +Y；旋转后坐标轴不会跟着车身旋转。
     * At reset, forward defines +X and left defines +Y; these axes stay fixed as the robot turns.
     */
    public Pose2D getPosition() {
        return device.getPosition();
    }

    /**
     * 返回最近更新的状态，如 READY、CALIBRATING 或 pod 连接故障。
     * Returns the latest status, such as READY, CALIBRATING, or a pod connection fault.
     */
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return device.getDeviceStatus();
    }
}
