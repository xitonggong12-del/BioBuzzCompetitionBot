package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 管理把球推到 Shooter 的小 Intake 电机。
 * Manages the small intake motor that pushes balls into the shooter.
 */
public class Transfer {

    // 名称 "transfer" 必须和 Driver Station 中的机器人配置一致。
    // The name "transfer" must match the robot configuration on Driver Station.
    private static final String DEVICE_NAME = "transfer";

    // 方向需实测：如果正转不是把球推向 Shooter，改成 REVERSE。
    // Verify on the robot: if forward does not push balls toward the shooter, change to REVERSE.
    private static final DcMotor.Direction DIRECTION = DcMotor.Direction.FORWARD;

    // 集中保存送球动力，以后调整速度时只需要修改这里。
    // Keep transfer power in one place so its speed can be adjusted easily.
    private static final double TRANSFER_POWER = 1.0;

    private final DcMotorEx motor;

    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, DEVICE_NAME);

        // 不使用 encoder；BRAKE 让球在松手时停在原位，不会自己滑进飞轮。
        // No encoder; BRAKE holds balls in place on release so they do not slide into the flywheel.
        motor.setDirection(DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /** 正转，把球推向 Shooter。 / Runs forward, pushing balls toward the shooter. */
    public void runForward() {
        motor.setPower(TRANSFER_POWER);
    }

    /** 反转，把球退回来。 / Runs in reverse, backing balls out. */
    public void runReverse() {
        motor.setPower(-TRANSFER_POWER);
    }

    /** 停止。 / Stops the motor. */
    public void stop() {
        motor.setPower(0.0);
    }

    /** 返回当前设定动力，供 Telemetry 显示。 / Returns commanded power for Telemetry. */
    public double getPower() {
        return motor.getPower();
    }
}
