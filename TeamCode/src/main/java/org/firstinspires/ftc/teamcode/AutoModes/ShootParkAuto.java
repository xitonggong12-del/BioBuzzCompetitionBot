package org.firstinspires.ftc.teamcode.AutoModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.odometrySystem.Pinpoint;
import org.firstinspires.ftc.teamcode.subSystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subSystem.Intake;
import org.firstinspires.ftc.teamcode.subSystem.Shooter;
import org.firstinspires.ftc.teamcode.subSystem.Transfer;

/**
 * 从 A1 出发的射球 + 停车自动：车头始终朝前，横移到 C1 射 4 个预装 POLLEN，再走 B 列开到 A5 停车。
 * Shoot-and-park autonomous from A1: the front always faces forward; strafe to C1 and launch the
 * 4 preloaded POLLEN, then drive up column B to park in A5.
 *
 * 起点：红方 A1（蓝方 F6），车尾贴观众侧场地墙，车头朝场地里面，车左右放在格子正中间。
 * Start: red A1 (blue F6), back on the audience-side wall, front facing into the field,
 * robot centered left-to-right in the tile.
 * 场地 180° 旋转对称，蓝方从 F6 跑同一个程序：横移到 D6 射球，停进 F2。
 * The field is 180° rotationally symmetric, so blue runs the same program from F6: strafe to D6, park in F2.
 *
 * 回程走 B 列而不贴左墙：左墙 A2/A3 交界处装着 FLOWER，贴墙往前开会撞上。
 * The return goes up column B instead of along the left wall: a FLOWER sits on the left wall at the
 * A2/A3 boundary and driving along the wall would hit it.
 *
 * 射球照 TeleOp 的做法：飞轮远档（最高转速 45%，闭环），转 2.5 秒后 Intake 和 Transfer 一起往里送。
 * Shooting follows TeleOp: flywheel on far (45% of top speed, closed loop); after 2.5 seconds
 * the intake and transfer feed together.
 *
 * 上场前：Odometry Test 复测方向；转台转到起点并确认车头朝前时对着 HIVE；装好 4 个 POLLEN；
 * INIT 后不要碰车，屏幕显示 READY 再按 START。
 * Before the match: recheck directions with Odometry Test; put the turret at its start and confirm it
 * faces the HIVE with the front forward; load 4 POLLEN; do not touch the robot after INIT,
 * and press START only after READY appears.
 */
@Autonomous(name = "A1 Shoot Park Auto", group = "Auto")
public class ShootParkAuto extends LinearOpMode {

    // 路线坐标相对起点，单位英寸：车头方向 +X，车左侧 +Y，与 Pinpoint 一致；朝向全程保持 0。
    // Route coordinates are relative to the start in inches: front is +X and left is +Y, matching Pinpoint;
    // heading stays 0 the whole way.
    // 一格 24 in；车宽按约 15 in 估算。
    // One tile is 24 in; the robot is assumed to be about 15 in wide.

    // 先往前离开观众墙，横移时才不会蹭墙。
    // First move off the audience wall so strafing does not rub it.
    private static final double LEAVE_X = 3.0;
    // C1：从 A1 往右两格。 / C1: two tiles right of A1.
    private static final double SHOOT_Y = -48.0;
    // B 列：车中心离左墙约 28 in，左边离开左墙的 FLOWER，右边离开 HIVE。
    // Column B: the robot center is about 28 in from the left wall, clear of the wall FLOWER and the HIVE.
    private static final double LANE_Y = -16.0;
    // A5 那一排：车中心约在场地墙往里 108 in，在 LOADING ZONE 前后范围的中间。
    // The A5 row: the robot center is about 108 in from the audience wall, mid-depth of the LOADING ZONE row.
    private static final double PARK_X = 100.0;
    // 回到起点的左右位置：车左边离左墙约 4.5 in，在 11 in 深的 LOADING ZONE 里，又不碰墙（保住 LEAVE）。
    // Back to the start's left-right position: the left side is ~4.5 in from the left wall, inside the
    // 11 in deep LOADING ZONE without touching the wall (keeping LEAVE).
    private static final double PARK_Y = 0.0;

    // 每段移动最多用这么久；被挡住时放弃这一段，继续下一步。合计约 19 秒，加上射球仍在 30 秒内。
    // Maximum time for each move; if blocked, give up on that move and continue. About 19 seconds in all,
    // still under 30 with shooting.
    private static final double LEAVE_TIMEOUT = 1.5;
    private static final double SHOOT_MOVE_TIMEOUT = 4.0;
    private static final double LANE_TIMEOUT = 3.0;
    private static final double FORWARD_TIMEOUT = 7.0;
    private static final double PARK_TIMEOUT = 3.0;

    // 和 driver 用远档时一样：从 START 起至少转 2.5 秒再送球，横移途中已经在加速。
    // Same as the driver on far: spin at least 2.5 seconds after START before feeding; it speeds up while strafing.
    private static final double SPIN_UP_SECONDS = 2.5;
    // 2.5 秒后还没到速，最多再等这么久，然后照样送球，免得卡住整个自动。
    // If not at speed after 2.5 seconds, wait at most this much longer, then feed anyway so the auto does not stall.
    private static final double MAX_EXTRA_SPIN_SECONDS = 1.0;
    // Intake 和 Transfer 一起往里送的时间；4 个没射完就加长。
    // How long the intake and transfer feed together; increase it if not all 4 are launched.
    private static final double FEED_SECONDS = 3.0;

    // 底盘控制参数与 Basic Auto 相同。 / Drive control constants match Basic Auto.
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double DRIVE_KP = 0.04;
    private static final double TURN_KP = 0.01;
    private static final double MAX_TURN_POWER = 0.3;
    private static final double MIN_TURN_POWER = 0.1;
    private static final double POSITION_TOLERANCE_IN = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 3.0;

    // 离目标反而比出发时远了这么多，说明底盘或里程计方向反了，立即停车。
    // Getting this much farther from the target than at the start means a drive or odometry
    // direction is reversed, so stop immediately.
    private static final double RUNAWAY_DISTANCE_IN = 6.0;
    private static final double RUNAWAY_HEADING_DEG = 45.0;

    // Pinpoint 没 READY 时的备用方案：按时间往前开一点拿 LEAVE，原地射球，不去停车。
    // Fallback when Pinpoint is not READY: drive forward briefly by time for LEAVE, shoot in place, skip parking.
    private static final double FALLBACK_DRIVE_POWER = 0.3;
    private static final double FALLBACK_DRIVE_SECONDS = 1.0;

    // reset 后至少等 0.5 秒，且设备报告 READY，读数才有效。
    // Readings are valid only 0.5 seconds after reset and once the device reports READY.
    private static final double RESET_SETTLE_SECONDS = 0.5;

    private Drivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Pinpoint pinpoint;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);

        // 车摆好后按 INIT：当前位置成为起点 (0, 0, 0)，校准期间不要碰车。
        // Press INIT after placing the robot: the current pose becomes the start (0, 0, 0);
        // do not touch it while calibrating.
        pinpoint.resetPosAndIMU();
        ElapsedTime resetTimer = new ElapsedTime();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Start: red A1 / blue F6, back on wall, centered in tile");
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            telemetry.addData("Pinpoint", status);
            if (isReady(status, resetTimer)) {
                // 位置应保持接近 0；数字在变说明车被碰了，需要重新 INIT。
                // The pose should stay near 0; if it changes, the robot was bumped and needs another INIT.
                telemetry.addLine("READY: Press START");
                addPoseTelemetry();
            } else {
                telemetry.addLine("Waiting for READY; keep robot still");
            }
            telemetry.addLine("Turret at start | 4 POLLEN loaded");
            telemetry.update();
            idle();
        }

        // 如果在 INIT 阶段停止，就不执行自动。
        // If stopped during INIT, skip the autonomous.
        if (isStopRequested()) {
            return;
        }

        // 一开始就让飞轮按远档闭环加速，横移到 C1 的途中就在转。
        // Spin up the flywheel at the far velocity, closed loop, right away so it spins while strafing to C1.
        ElapsedTime autoTimer = new ElapsedTime();
        shooter.setTargetVelocity(Shooter.FAR_VELOCITY_TPS);

        pinpoint.update();
        if (!isReady(pinpoint.getDeviceStatus(), resetTimer)) {
            // 没有可靠位置就不能走路线：按时间离墙，原地射球，不去停车。
            // Without a reliable pose the route cannot run: leave the wall by time, shoot in place, and skip parking.
            drivetrain.drive(FALLBACK_DRIVE_POWER, 0.0, 0.0);
            waitSeconds(FALLBACK_DRIVE_SECONDS, "Fallback: leave by time");
            drivetrain.stop();
            shootPreloads(autoTimer);
            finish("Pinpoint not READY: LEAVE + shoot only");
            return;
        }

        // 每一段都是直线；朝向保持 0，车头一直朝前。
        // Each move is a straight line; heading stays 0 so the front always faces forward.
        if (!driveTo("Leave wall", LEAVE_X, 0.0, LEAVE_TIMEOUT)
                || !driveTo("Strafe to C1", LEAVE_X, SHOOT_Y, SHOOT_MOVE_TIMEOUT)) {
            finish("STOPPED: moving away from target, check directions");
            return;
        }
        shootPreloads(autoTimer);
        if (!driveTo("Strafe to column B", LEAVE_X, LANE_Y, LANE_TIMEOUT)
                || !driveTo("Forward to A5 row", PARK_X, LANE_Y, FORWARD_TIMEOUT)
                || !driveTo("Strafe into zone", PARK_X, PARK_Y, PARK_TIMEOUT)) {
            finish("STOPPED: moving away from target, check directions");
            return;
        }
        finish("Done");
    }

    /**
     * 用 Pinpoint 闭环开到目标位置，朝向保持 0；超时就放弃这一段，返回 true 继续下一步。
     * Drives to the target position with Pinpoint feedback, holding heading 0; on timeout it gives up
     * on this move and returns true so the next step continues.
     * 返回 false 表示应结束自动：越走越远（方向反了）或按了 STOP。
     * Returns false when the autonomous should end: the robot is moving away (a direction is reversed)
     * or STOP was pressed.
     */
    private boolean driveTo(String step, double targetX, double targetY, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double startDistance = Double.NaN;
        double startHeadingError = Double.NaN;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double errorX = targetX - pose.getX(DistanceUnit.INCH);
            double errorY = targetY - pose.getY(DistanceUnit.INCH);
            double distance = Math.hypot(errorX, errorY);
            double headingError = AngleUnit.normalizeDegrees(-heading);

            if (distance <= POSITION_TOLERANCE_IN && Math.abs(headingError) <= HEADING_TOLERANCE_DEG) {
                drivetrain.stop();
                return true;
            }

            // 记住出发时的误差；之后误差明显变大，说明车在往反方向走。
            // Remember the starting error; if it grows clearly after that, the robot is going the wrong way.
            if (Double.isNaN(startDistance)) {
                startDistance = distance;
                startHeadingError = Math.abs(headingError);
            }
            if (distance > startDistance + RUNAWAY_DISTANCE_IN
                    || Math.abs(headingError) > startHeadingError + RUNAWAY_HEADING_DEG) {
                drivetrain.stop();
                telemetry.log().add(step + ": moving away from target");
                return false;
            }

            // 把场地方向的误差换算成车身方向：车头方向的分量和车左侧的分量。
            // Convert the field-frame error into the robot frame: the part toward the front and toward the left.
            double headingRad = Math.toRadians(heading);
            double forwardError = errorX * Math.cos(headingRad) + errorY * Math.sin(headingRad);
            double leftError = -errorX * Math.sin(headingRad) + errorY * Math.cos(headingRad);

            // 远时快、近时慢，但不低于推得动的最小动力；再按误差方向把动力分到前后和横移。
            // Fast when far and slow when close, but never below the minimum power that moves the robot;
            // the power is then split between forward and strafe along the error direction.
            double driveScale = 0.0;
            if (distance > POSITION_TOLERANCE_IN) {
                driveScale = Range.clip(distance * DRIVE_KP, MIN_DRIVE_POWER, MAX_DRIVE_POWER) / distance;
            }
            double turnPower = 0.0;
            if (Math.abs(headingError) > HEADING_TOLERANCE_DEG) {
                turnPower = Range.clip(Math.abs(headingError) * TURN_KP, MIN_TURN_POWER, MAX_TURN_POWER);
            }

            // Drivetrain 横移向右为正、旋转顺时针为正，和里程计的向左、逆时针为正相反，所以取负。
            // Drivetrain strafe is positive right and turn is positive clockwise, opposite to odometry's
            // left and counter-clockwise, so both are negated.
            drivetrain.drive(
                    forwardError * driveScale,
                    -leftError * driveScale,
                    -Math.signum(headingError) * turnPower
            );

            telemetry.addData("Step", step);
            telemetry.addData("Target", "X %.1f | Y %.1f", targetX, targetY);
            addPoseTelemetry();
            telemetry.update();
        }

        drivetrain.stop();
        if (opModeIsActive()) {
            telemetry.log().add(step + ": timed out");
        }
        return opModeIsActive();
    }

    /**
     * 等飞轮从 START 起转满 2.5 秒并到速，再用 Intake 和 Transfer 一起把 4 个 POLLEN 送进飞轮，最后全部停下。
     * Waits until the flywheel has spun 2.5 seconds since START and reached speed, feeds the 4 POLLEN
     * with the intake and transfer together, then stops all three.
     */
    private void shootPreloads(ElapsedTime autoTimer) {
        while (opModeIsActive() && autoTimer.seconds() < SPIN_UP_SECONDS + MAX_EXTRA_SPIN_SECONDS) {
            shooter.update();
            if (autoTimer.seconds() >= SPIN_UP_SECONDS && shooter.isReady()) {
                break;
            }
            telemetry.addData("Step", "Spin up flywheel");
            telemetry.addData("Flywheel", "%.0f / %.0f RPM",
                    Shooter.toRpm(shooter.getVelocity()), Shooter.toRpm(Shooter.FAR_VELOCITY_TPS));
            telemetry.update();
        }
        if (!opModeIsActive()) {
            return;
        }

        // 和 TeleOp 里 driver 同时按 gamepad1 RB 和 gamepad2 X 一样。
        // Same as the driver holding gamepad1 RB and gamepad2 X together in TeleOp.
        intake.runForward();
        transfer.runForward();
        waitSeconds(FEED_SECONDS, "Feed POLLEN");
        intake.stop();
        transfer.stop();
        shooter.stop();
    }

    /**
     * 等待指定秒数并显示当前步骤；返回 false 表示中途按了 STOP。
     * Waits for the given seconds while showing the current step; returns false if STOP was pressed.
     */
    private boolean waitSeconds(double seconds, String step) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Step", step);
            telemetry.update();
            idle();
        }
        return opModeIsActive();
    }

    /**
     * 停下所有机构，并一直显示结果和最终位置直到自动结束，方便对照车实际停的位置调参数。
     * Stops every mechanism and shows the result and final pose until the autonomous ends,
     * to compare with where the robot actually stopped when tuning.
     */
    private void finish(String result) {
        drivetrain.stop();
        intake.stop();
        transfer.stop();
        shooter.stop();
        while (opModeIsActive()) {
            pinpoint.update();
            telemetry.addData("Result", result);
            addPoseTelemetry();
            telemetry.update();
            idle();
        }
    }

    /** 显示当前位置（英寸）和朝向（度）。 / Shows the current position in inches and heading in degrees. */
    private void addPoseTelemetry() {
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData(
                "Pose",
                "X %.1f | Y %.1f | H %.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES)
        );
    }

    /**
     * 设备 READY 且 reset 后已过最短等待时间，读数才有效。
     * Readings are valid only when the device is READY and the minimum reset wait has passed.
     */
    private boolean isReady(GoBildaPinpointDriver.DeviceStatus status, ElapsedTime resetTimer) {
        return status == GoBildaPinpointDriver.DeviceStatus.READY
                && resetTimer.seconds() >= RESET_SETTLE_SECONDS;
    }
}
