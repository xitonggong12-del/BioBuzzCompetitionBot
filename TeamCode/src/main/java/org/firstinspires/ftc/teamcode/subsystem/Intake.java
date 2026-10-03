package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 管理 Intake 电机的初始化和运行方向。
 * Manages initialization and movement direction for the intake motor.
 */
public class Intake {

    // 集中保存 Intake 动力，以后调整速度时只需要修改这里。
    // Keep intake power in one place so its speed can be adjusted easily.
    private static final double INTAKE_POWER = 1.0;

    private final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        // 名称 "intake" 必须和 Driver Station 中的机器人配置一致。
        // The name "intake" must match the robot configuration on Driver Station.
        motor = hardwareMap.get(DcMotorEx.class, "intake");

        // Intake 当前不使用 encoder；BRAKE 会在动力归零后主动保持停止。
        // The intake currently uses no encoder; BRAKE actively holds it when power is zero.
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    /** 以设定动力正转 Intake。 / Runs the intake forward at the configured power. */
    public void runForward() {
        motor.setPower(INTAKE_POWER);
    }

    /** 以设定动力反转 Intake。 / Runs the intake in reverse at the configured power. */
    public void runReverse() {
        motor.setPower(-INTAKE_POWER);
    }

    /** 停止 Intake。 / Stops the intake. */
    public void stop() {
        motor.setPower(0.0);
    }

    /** 返回当前设定动力，供 Telemetry 显示。 / Returns commanded power for Telemetry. */
    public double getPower() {
        return motor.getPower();
    }
}
