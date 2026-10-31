package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 管理 Intake 左边的限位舵机：收起 Intake、放到底，或停在只让小 POLLEN 进、不让大 NECTAR 进的高度。
 * Manages the stop servo on the left of the intake: holds it stowed, lowers it fully, or holds it
 * at a height that lets small POLLEN in but keeps large NECTAR out.
 *
 * 舵机是 goBILDA Dual Mode Servo，必须切到普通位置模式，Driver Station 中配置为 Servo。
 * The servo is a goBILDA Dual Mode Servo; it must be in standard position mode and configured as Servo on Driver Station.
 *
 * 全部手动：drop() 放到底，gap() 抬到卡大球的高度，不会自己切换。
 * Fully manual: drop() lowers it fully, gap() lifts it to the NECTAR-blocking height; nothing switches on its own.
 */
public class IntakeGate {

    // 名称必须和 Driver Station 中的机器人配置一致。
    // The name must match the robot configuration on Driver Station.
    public static final String DEVICE_NAME = "gateLeft";

    // 改方向后三个位置都要用 Intake Gate Test 重新测。
    // After changing the direction, remeasure all three positions with Intake Gate Test.
    public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;

    // 三个位置都是暂定值，必须用 Intake Gate Test 实测后填进来。
    // All three positions are placeholders and must be measured with Intake Gate Test.
    // STOWED：收起 Intake。 / STOWED: holds the intake up.
    private static final double STOWED = 0.0;
    // DROP：舵机臂完全让开，Intake 落到底。 / DROP: the arm is fully clear and the intake falls all the way.
    private static final double DROP = 0.5;
    // GAP：把 Intake 抬起一点，开口只够 POLLEN 进。 / GAP: lifts the intake slightly so only POLLEN fits.
    private static final double GAP = 0.4;

    private enum State { STOWED, DROP, GAP }

    private final Servo servo;
    private State state;

    public IntakeGate(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, DEVICE_NAME);
        servo.setDirection(DIRECTION);
        stow();
    }

    /** 收起 Intake。 / Holds the intake stowed. */
    public void stow() {
        servo.setPosition(STOWED);
        state = State.STOWED;
    }

    /** 放到底，大小球都能进。 / Lowers the intake fully; both ball sizes fit. */
    public void drop() {
        servo.setPosition(DROP);
        state = State.DROP;
    }

    /** 抬到只让 POLLEN 进的高度。 / Lifts the intake to the POLLEN-only height. */
    public void gap() {
        servo.setPosition(GAP);
        state = State.GAP;
    }

    /** 当前状态名，供 Telemetry 显示。 / Current state name for Telemetry. */
    public String getStateName() {
        return state.name();
    }
}
