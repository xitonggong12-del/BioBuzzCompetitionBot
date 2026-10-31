package org.firstinspires.ftc.teamcode.AopModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometrySystem.Pinpoint;
import org.firstinspires.ftc.teamcode.subSystem.AprilTagCamera;
import org.firstinspires.ftc.teamcode.subSystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subSystem.Intake;
import org.firstinspires.ftc.teamcode.subSystem.IntakeGate;
import org.firstinspires.ftc.teamcode.subSystem.Shooter;
import org.firstinspires.ftc.teamcode.subSystem.Transfer;
import org.firstinspires.ftc.teamcode.subSystem.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/**
 * 比赛用 TeleOp。
 * Competition TeleOp.
 *
 * gamepad1：左摇杆 前后/横移 | 右摇杆 旋转 | RB Intake 正转 | LB Intake 反转 | X 送球正转 | Y 送球反转
 * | 十字键下 Intake 放到底 | 十字键上 Intake 抬到卡大球高度 | BACK 里程计清零（停稳后）
 * gamepad1: left stick drive/strafe | right stick turn | RB intake in | LB intake out
 * | X transfer forward | Y transfer reverse | D-pad down lower intake fully
 * | D-pad up lift intake to NECTAR-blocking height | BACK reset odometry (while still)
 *
 * gamepad2：RB 飞轮发射 | LB 飞轮反转 | 十字键上 远档 | 十字键下 近档 | RT 转台顺时针 | LT 转台逆时针
 * | A Intake 正转 | B Intake 反转 | X 送球正转 | Y 送球反转 | BACK 转台重新对准起点
 * gamepad2: RB flywheel launch | LB flywheel reverse | D-pad up far | D-pad down near | RT turret clockwise
 * | LT turret counter-clockwise | A intake in | B intake out | X transfer forward | Y transfer reverse
 * | BACK re-zero turret at start
 *
 * 自动瞄准：gamepad2 十字键右 打开、十字键左 关闭；打开时转台自动对准己方得分区的 AprilTag 簇，
 * 按 RT/LT 手动转会临时接管。INIT 期间 gamepad2 X 选蓝方、B 选红方。
 * Auto-aim: gamepad2 D-pad right turns it on, D-pad left off; while on, the turret centers on the
 * alliance scoring AprilTag cluster, and RT/LT take over manually. During INIT gamepad2 X picks blue, B picks red.
 *
 * Intake 和送球两个手柄都能控制：有人要正转、没人要反转时正转，反过来就反转，两个方向都有人要时停止。
 * Both gamepads control the intake and transfer: forward when only forward is requested, reverse when only
 * reverse is requested, and stop when both directions are requested.
 *
 * Intake 限位舵机：INIT 时自动收起，START 后不自动动，只听 gamepad1 十字键下/上。
 * The intake stop servo is stowed automatically during INIT; after START it only moves on gamepad1 D-pad down/up.
 *
 * 上场前：转台摆到起点再 INIT；INIT 期间不要动车，里程计在校准。
 * Before the match: put the turret at the start before INIT; do not move the robot during INIT while odometry calibrates.
 */
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleOp extends LinearOpMode {

    // 飞轮反转用全动力开环，用来清卡球。
    // Flywheel reverse uses full open-loop power to clear jams.
    private static final double FLYWHEEL_REVERSE_POWER = 1.0;

    // 忽略摇杆和扳机的微小读数，防止松手时机构缓慢移动。
    // Ignore tiny stick and trigger readings so nothing creeps when untouched.
    private static final double JOYSTICK_DEADBAND = 0.05;

    // 里程计清零后至少等待 0.5 秒，并且设备报告 READY，读数才有效。
    // After an odometry reset, readings are valid only after 0.5 seconds and a READY status.
    private static final double ODOMETRY_SETTLE_SECONDS = 0.5;

    // 自动瞄准假设摄像头装在转台上、跟着转台转；装在底盘上的话这套比例控制不适用。
    // Auto-aim assumes the camera is mounted on the turret and turns with it; it does not work for a chassis mount.
    // 每偏 1° 给的转台动力； 来回晃就调小，太慢就调大。
    // Turret power per degree of error; lower it if the turret oscillates, raise it if it is too slow.
    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_POWER = 0.4;
    // 偏差小于这个角度就算对准，停下不再修正。
    // Within this many degrees counts as aimed; stop correcting.
    private static final double AIM_TOLERANCE_DEG = 1.5;

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        // 创建时就转到 STOWED，INIT 期间一直把 Intake 收着。
        // Turns to STOWED on creation and holds the intake up through INIT.
        IntakeGate intakeGate = new IntakeGate(hardwareMap);

        // Pinpoint 没配置或没插好时为 null，只是不显示里程计，其他功能照常。
        // Pinpoint is null when missing from the configuration; odometry is hidden and everything else still works.
        Pinpoint pinpoint;
        try {
            pinpoint = new Pinpoint(hardwareMap);
            pinpoint.resetPosAndIMU();
        } catch (IllegalArgumentException e) {
            pinpoint = null;
        }
        ElapsedTime odometryTimer = new ElapsedTime();

        // 摄像头没配置时为 null，只是不能自动瞄准，其他功能照常。
        // The camera is null when missing from the configuration; auto-aim is unavailable and everything else works.
        AprilTagCamera camera;
        try {
            camera = new AprilTagCamera(hardwareMap);
        } catch (IllegalArgumentException e) {
            camera = null;
        }

        // 默认红方，INIT 期间用 gamepad2 X/B 改。 / Defaults to red; change with gamepad2 X/B during INIT.
        boolean blueAlliance = false;

        // START 前持续刷新，便于确认里程计校准完成。
        // Refresh before START so odometry calibration can be confirmed.
        while (!isStarted() && !isStopRequested()) {
            if (gamepad2.x) {
                blueAlliance = true;
            } else if (gamepad2.b) {
                blueAlliance = false;
            }
            telemetry.addLine("Turret at START before INIT!");
            telemetry.addData("Alliance", "%s (gamepad2 X blue / B red)", blueAlliance ? "BLUE" : "RED");
            telemetry.addData("Camera", camera != null ? "OK" : "not connected (no auto-aim)");
            if (pinpoint != null) {
                pinpoint.update();
            }
            addOdometryTelemetry(pinpoint, odometryTimer);
            telemetry.update();
            idle();
        }

        // 如果在开始前按下停止，就不要进入主循环。
        // Do not enter the main loop if Stop was requested before starting.
        if (isStopRequested()) {
            return;
        }

        // 飞轮默认远档；记录 START 时的 BACK 状态，已经按住的不算新按键。
        // The flywheel defaults to far; remember BACK at START so an already-held button is not a new press.
        boolean farMode = true;
        boolean previousOdometryReset = gamepad1.back;
        boolean autoAim = false;
        String targetName = blueAlliance ? AprilTagCamera.BLUE_SCORING : AprilTagCamera.RED_SCORING;

        while (opModeIsActive()) {
            // ===== gamepad1：底盘 =====
            // ===== gamepad1: drivetrain =====
            double forward = applyDeadband(-gamepad1.left_stick_y);
            double strafe = applyDeadband(gamepad1.left_stick_x);
            double turn = applyDeadband(gamepad1.right_stick_x);
            drivetrain.drive(forward, strafe, turn);

            // ===== Intake：gamepad1 RB/LB，gamepad2 A/B =====
            // ===== Intake: gamepad1 RB/LB, gamepad2 A/B =====
            boolean intakeIn = gamepad1.right_bumper || gamepad2.a;
            boolean intakeOut = gamepad1.left_bumper || gamepad2.b;
            if (intakeIn && !intakeOut) {
                intake.runForward();
            } else if (intakeOut && !intakeIn) {
                intake.runReverse();
            } else {
                intake.stop();
            }

            // ===== gamepad1：Intake 限位舵机，十字键下放到底、上抬到卡大球高度，按一下一直保持 =====
            // ===== gamepad1: intake stop servo, D-pad down lowers fully and up lifts to the NECTAR-blocking
            // height, kept until changed =====
            if (gamepad1.dpad_down) {
                intakeGate.drop();
            } else if (gamepad1.dpad_up) {
                intakeGate.gap();
            }

            // ===== 送球：两个手柄的 X/Y =====
            // ===== Transfer: X/Y on either gamepad =====
            boolean transferIn = gamepad1.x || gamepad2.x;
            boolean transferOut = gamepad1.y || gamepad2.y;
            if (transferIn && !transferOut) {
                transfer.runForward();
            } else if (transferOut && !transferIn) {
                transfer.runReverse();
            } else {
                transfer.stop();
            }

            // ===== gamepad2：飞轮档位，十字键上远、下近，按一下一直保持 =====
            // ===== gamepad2: flywheel range, D-pad up far and down near, kept until changed =====
            if (gamepad2.dpad_up) {
                farMode = true;
            } else if (gamepad2.dpad_down) {
                farMode = false;
            }
            double targetVelocity = farMode ? Shooter.FAR_VELOCITY_TPS : Shooter.NEAR_VELOCITY_TPS;

            // ===== gamepad2：飞轮，RB 按档位闭环发射、LB 开环反转；都不按或同时按时滑行停止 =====
            // ===== gamepad2: flywheel, RB launches at the range velocity, LB reverses open loop;
            // coast to a stop with none or both held =====
            boolean launchHeld = gamepad2.right_bumper;
            boolean reverseHeld = gamepad2.left_bumper;
            String flywheelMode;
            if (launchHeld && !reverseHeld) {
                shooter.setTargetVelocity(targetVelocity);
                flywheelMode = "LAUNCH";
            } else if (reverseHeld && !launchHeld) {
                shooter.setOpenLoopPower(-FLYWHEEL_REVERSE_POWER);
                flywheelMode = "REVERSE";
            } else {
                shooter.stop();
                flywheelMode = "STOP";
            }

            // ===== gamepad2：转台，限制在 0°–270° 内（由 Turret 负责） =====
            // ===== gamepad2: turret, limited to 0°–270° (handled by Turret) =====
            // 转台位置靠估算，漂了就手动转回起点再按 BACK。
            // The turret position is estimated; if it drifts, turn it back to the start by hand and press BACK.
            if (gamepad2.back) {
                turret.resetToStart();
            }
            if (gamepad2.dpad_right) {
                autoAim = true;
            } else if (gamepad2.dpad_left) {
                autoAim = false;
            }
            AprilTagClusterDetection target = camera != null ? camera.getCluster(targetName) : null;
            double clockwise = applyDeadband(gamepad2.right_trigger - gamepad2.left_trigger);
            String aimStatus;
            if (clockwise != 0.0 || !autoAim) {
                aimStatus = autoAim ? "MANUAL OVERRIDE" : "OFF";
            } else if (target == null) {
                // 看不到目标就停住，不乱找，免得转到限位。
                // Hold still without a target instead of searching, so it does not run into a limit.
                aimStatus = "NO TARGET";
            } else if (Math.abs(target.ftcPose.bearing) <= AIM_TOLERANCE_DEG) {
                aimStatus = "LOCKED";
            } else {
                // bearing 为正是目标在左边，要逆时针（负动力）转；方向不对就把负号去掉。
                // Positive bearing means the target is to the left, so turn counter-clockwise (negative power);
                // drop the minus sign if it turns the wrong way.
                clockwise = Range.clip(-target.ftcPose.bearing * AIM_KP, -AIM_MAX_POWER, AIM_MAX_POWER);
                aimStatus = "AIMING";
            }
            turret.rotate(clockwise);

            // ===== gamepad1：BACK 里程计清零，只在按下那一下触发 =====
            // ===== gamepad1: BACK resets odometry, triggered once per press =====
            boolean odometryResetHeld = gamepad1.back;
            boolean odometryResetPressed = odometryResetHeld && !previousOdometryReset;
            previousOdometryReset = odometryResetHeld;
            if (pinpoint != null) {
                pinpoint.update();
                if (odometryResetPressed && isOdometryReady(pinpoint, odometryTimer)) {
                    // 操作者必须先停稳再按；程序不会自动判断车是否静止。
                    // The operator must stop the robot before pressing; this code does not detect stillness.
                    pinpoint.resetPosAndIMU();
                    odometryTimer.reset();
                }
            }

            telemetry.addData(
                    "Flywheel",
                    "%s | %s | target %.0f / actual %.0f RPM",
                    flywheelMode,
                    farMode ? "FAR" : "NEAR",
                    Shooter.toRpm(targetVelocity),
                    Shooter.toRpm(shooter.getVelocity())
            );
            telemetry.addData(
                    "Turret",
                    "%.0f° (est)%s",
                    turret.getEstimatedAngle(),
                    turret.isAtLimit() ? " | LIMIT" : ""
            );
            if (target != null) {
                telemetry.addData(
                        "Auto-aim",
                        "%s | %s | %.1f° | %.0f in",
                        aimStatus,
                        targetName,
                        target.ftcPose.bearing,
                        target.ftcPose.range
                );
            } else {
                telemetry.addData("Auto-aim", "%s | %s not seen", aimStatus, targetName);
            }
            telemetry.addData("Intake gate", intakeGate.getStateName());
            addOdometryTelemetry(pinpoint, odometryTimer);
            telemetry.update();
        }

        // OpMode 结束时停止所有机构，飞轮滑行停下。
        // Stop every mechanism when the OpMode ends; the flywheel coasts down.
        drivetrain.stop();
        intake.stop();
        transfer.stop();
        shooter.stop();
        turret.stop();
        if (camera != null) {
            camera.close();
        }
    }

    /**
     * 显示里程计状态；未就绪时不显示位置，避免把校准中的读数当真。
     * Shows odometry; hides the pose while not ready so calibration readings are not mistaken for valid ones.
     * 调用前先 update，这里只读取。
     * Call update() first; this only reads.
     */
    private void addOdometryTelemetry(Pinpoint pinpoint, ElapsedTime odometryTimer) {
        if (pinpoint == null) {
            telemetry.addLine("Odometry not connected");
            return;
        }
        if (isOdometryReady(pinpoint, odometryTimer)) {
            Pose2D pose = pinpoint.getPosition();
            telemetry.addData(
                    "Odometry",
                    "X %.0f mm | Y %.0f mm | %.1f°",
                    pose.getX(DistanceUnit.MM),
                    pose.getY(DistanceUnit.MM),
                    pose.getHeading(AngleUnit.DEGREES)
            );
        } else {
            telemetry.addData("Odometry", "%s (keep still)", pinpoint.getDeviceStatus());
        }
    }

    /**
     * 设备 READY 且清零后过了最短等待时间，读数才有效。
     * Readings are valid only when the device is READY and the minimum wait after a reset has passed.
     */
    private boolean isOdometryReady(Pinpoint pinpoint, ElapsedTime odometryTimer) {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && odometryTimer.seconds() >= ODOMETRY_SETTLE_SECONDS;
    }

    /**
     * 将死区范围内的输入变为零。
     * Converts input inside the deadband to zero.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }
}
