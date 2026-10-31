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

/**
 * 最基础的自动：离墙拿 LEAVE → 射出 4 个预装 POLLEN（不瞄准）→ 停进 LOADING ZONE 拿 PARK。
 * Basic autonomous: leave the wall for LEAVE → launch the 4 preloaded POLLEN (no aiming)
 * → park in the LOADING ZONE for PARK.
 *
 * 两个起点，INIT 时用 gamepad1 十字键选择；赛前和队友商量好谁用哪个，两台车各停 LOADING ZONE 的一半。
 * Two starts, chosen with the gamepad1 D-pad during INIT; agree with the partner before the match
 * which one each robot uses, so each parks in half of the LOADING ZONE.
 * 两个起点都是车尾贴自己的联盟墙、车头朝场地中间：
 * Both starts have the back against our alliance wall and the front facing the middle of the field:
 * - WALL：红方 A3 格（蓝方 F4），车左侧对齐与 LOADING ZONE 那边相邻格之间的地垫缝；停区域下半（远离角落那半）。
 *   WALL: red tile A3 (blue F4), left side on the seam toward the LOADING ZONE; parks in the half away from the corner.
 * - CORNER：红方 A6 格（蓝方 F1），车左侧贴角落的场地墙；停区域上半（靠近角落那半）。
 *   CORNER: red tile A6 (blue F1), left side against the corner's wall; parks in the half near the corner.
 * 场地 180° 旋转对称，所以蓝方用同一个程序，只是格子换成括号里的。
 * The field is 180° rotationally symmetric, so blue runs the same program from the tiles in parentheses.
 *
 * 上场前：转台摆正，装好 4 个 POLLEN；INIT 后不要碰车，屏幕显示 READY 再按 START。
 * Before the match: center the turret and load 4 POLLEN; do not touch the robot after INIT,
 * and press START only after READY appears.
 */
@Autonomous(name = "Basic Auto", group = "Auto")
public class BasicAuto extends LinearOpMode {

    /**
     * 一个起点对应的路线：射球点和停车点。
     * The route for one start: the shooting spot and the parking spot.
     * 坐标都相对起点，单位英寸和度：车头方向 +X，车左侧 +Y，逆时针为正，与 Pinpoint 一致。
     * Coordinates are relative to the start in inches and degrees: front is +X, left is +Y,
     * and counter-clockwise is positive, matching Pinpoint.
     */
    private static final class Route {
        final String name;
        final double shootX;
        final double shootY;
        final double shootHeading;
        final double parkX;
        final double parkY;

        Route(String name, double shootX, double shootY, double shootHeading, double parkX, double parkY) {
            this.name = name;
            this.shootX = shootX;
            this.shootY = shootY;
            this.shootHeading = shootHeading;
            this.parkX = parkX;
            this.parkY = parkY;
        }
    }

    // 两条路线的数字按车宽约 15 in 估算：车中心离 A3/A4 缝或顶墙约 7.5 in。
    // Both routes assume a robot about 15 in wide: the center is about 7.5 in from the A3/A4 seam or the top wall.
    // 射球口如果在车尾，把两条路线的射球朝向都加 180，车会原地掉头再射。
    // If the shooter launches out the back, add 180 to both shooting headings so the robot turns around first.
    // 停车点车尾离墙 5 in：LOADING ZONE 只有 11 in 深，车要有一部分在区域里，但碰墙会丢掉 LEAVE 的 3 分。
    // Parking leaves the back 5 in from the wall: the zone is only 11 in deep, part of the robot must be inside,
    // and touching the wall loses LEAVE's 3 points.
    // 两车的停车点中心相距 16 in，车宽超过 15 in 时要和队友确认不会撞。
    // The two parking centers are 16 in apart; check with the partner if either robot is wider than 15 in.

    // WALL：往前一格（24 in）离墙，正对 HIVE 朝上的 CELL 射球；再往左 36 in 停区域下半。
    // WALL: one tile (24 in) forward off the wall, shooting straight at the HIVE's upward CELL;
    // then 36 in left to park in the lower half of the zone.
    private static final Route WALL_ROUTE = new Route("WALL (red A3 / blue F4)", 24.0, 0.0, 0.0, 5.0, 36.0);

    // CORNER：斜着离开两面墙，向右转约 65° 大致朝 HIVE 射球（没有精确瞄准）；再停区域上半。
    // CORNER: move diagonally off both walls, turn about 65° right to roughly face the HIVE (not precisely aimed),
    // then park in the upper half of the zone.
    // 射球点先离顶墙 12 in，否则原地转向时车角会扫到墙。
    // The shooting spot is first 12 in off the top wall, or the robot's corner would hit the wall while turning.
    private static final Route CORNER_ROUTE = new Route("CORNER (red A6 / blue F1)", 24.0, -12.0, -65.0, 5.0, -20.0);

    // 每段移动最多用这么久；被挡住时放弃这一段，继续下一步。
    // Maximum time for each move; if blocked, give up on that move and continue with the next step.
    private static final double SHOOT_MOVE_TIMEOUT = 4.0;
    private static final double PARK_MOVE_TIMEOUT = 6.0;

    // 飞轮从 START 起至少转这么久才送球；开车途中已经在加速。
    // The flywheel spins at least this long after START before feeding; it speeds up during the drive.
    private static final double SPIN_UP_SECONDS = 1.5;
    // Intake 往里送球的时间；4 个没射完就加长。
    // How long the intake feeds inward; increase it if not all 4 are launched.
    private static final double FEED_SECONDS = 3.0;

    // 底盘最大动力；先用 0.5 慢速测试，路线可靠后再调高。
    // Maximum drive power; test slowly at 0.5 first and raise it once the route is reliable.
    private static final double MAX_DRIVE_POWER = 0.5;
    // 低于这个动力车推不动；接近目标时至少用它，避免停在半路。
    // Below this power the robot does not move; use at least this near the target so it does not stall short.
    private static final double MIN_DRIVE_POWER = 0.15;
    // 每差 1 in 给多少动力：差 12 in 以上就是最大动力。
    // Power per inch of error: 12 in or more of error gives maximum power.
    private static final double DRIVE_KP = 0.04;

    // 旋转同理：每差 1° 给多少动力，以及最大和最小旋转动力。
    // Turning works the same way: power per degree of error, plus maximum and minimum turn power.
    private static final double TURN_KP = 0.01;
    private static final double MAX_TURN_POWER = 0.3;
    private static final double MIN_TURN_POWER = 0.1;

    // 位置和朝向误差都小于这些值才算到达。
    // Arrival requires both the position and heading errors to be within these values.
    private static final double POSITION_TOLERANCE_IN = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 3.0;

    // 离目标反而比出发时远了这么多，说明底盘或里程计方向反了，立即停车。
    // Getting this much farther from the target than at the start means a drive or odometry
    // direction is reversed, so stop immediately.
    private static final double RUNAWAY_DISTANCE_IN = 6.0;
    private static final double RUNAWAY_HEADING_DEG = 45.0;

    // Pinpoint 没 READY 时的备用方案：按时间往前开一点，至少拿到 LEAVE。
    // Fallback when Pinpoint is not READY: drive forward briefly by time to at least get LEAVE.
    private static final double FALLBACK_DRIVE_POWER = 0.3;
    private static final double FALLBACK_DRIVE_SECONDS = 1.0;

    // 与 Odometry Test 相同：reset 后至少等 0.5 秒，且设备报告 READY，读数才有效。
    // Same as Odometry Test: readings are valid only 0.5 seconds after reset and once the device reports READY.
    private static final double RESET_SETTLE_SECONDS = 0.5;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Pinpoint pinpoint;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);

        // 车摆好后按 INIT：当前位置成为起点 (0, 0, 0)，校准期间不要碰车。
        // Press INIT after placing the robot: the current pose becomes the start (0, 0, 0);
        // do not touch it while calibrating.
        pinpoint.resetPosAndIMU();
        ElapsedTime resetTimer = new ElapsedTime();

        // 默认 WALL；INIT 期间十字键下选 WALL、上选 CORNER，START 时以最后的选择为准。
        // Defaults to WALL; during INIT, D-pad down picks WALL and up picks CORNER; the last choice at START is used.
        Route route = WALL_ROUTE;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_down) {
                route = WALL_ROUTE;
            } else if (gamepad1.dpad_up) {
                route = CORNER_ROUTE;
            }
            telemetry.addData("Start", route.name);
            telemetry.addLine("D-pad: down = WALL | up = CORNER");

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
            telemetry.addLine("Back on wall | turret centered | 4 POLLEN loaded");
            telemetry.update();
            idle();
        }

        // 如果在 INIT 阶段停止，就不执行自动。
        // If stopped during INIT, skip the autonomous.
        if (isStopRequested()) {
            return;
        }

        // 一开始就让飞轮按 TeleOp 远档闭环加速，开到射球点时基本已经到速。
        // Spin up the flywheel right away at the TeleOp far velocity, closed loop, so it is nearly at speed
        // when the robot reaches the shooting spot.
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

        if (!driveTo("Drive to shoot", route.shootX, route.shootY, route.shootHeading, SHOOT_MOVE_TIMEOUT)) {
            finish("STOPPED: moving away from target, check directions");
            return;
        }
        shootPreloads(autoTimer);
        // 停车时转回朝向 0，车尾对着墙，PARK 的离墙距离才准。
        // Turn back to heading 0 when parking so the back faces the wall and the wall clearance is correct.
        if (!driveTo("Park", route.parkX, route.parkY, 0.0, PARK_MOVE_TIMEOUT)) {
            finish("STOPPED: moving away from target, check directions");
            return;
        }
        finish("Done");
    }

    /**
     * 用 Pinpoint 闭环开到目标位置和朝向；超时就放弃这一段，返回 true 继续下一步。
     * Drives to the target position and heading with Pinpoint feedback; on timeout it gives up
     * on this move and returns true so the next step continues.
     * 返回 false 表示应结束自动：越走越远（方向反了）或按了 STOP。
     * Returns false when the autonomous should end: the robot is moving away (a direction is reversed)
     * or STOP was pressed.
     */
    private boolean driveTo(String step, double targetX, double targetY, double targetHeading,
                            double timeoutSeconds) {
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
            double headingError = AngleUnit.normalizeDegrees(targetHeading - heading);

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
            telemetry.addData("Target", "X %.1f | Y %.1f | H %.1f", targetX, targetY, targetHeading);
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
     * 等飞轮转够时间，再用 Intake 把 4 个 POLLEN 送进飞轮，最后停下两者。
     * Waits until the flywheel has spun long enough, feeds the 4 POLLEN into it with the intake,
     * then stops both.
     */
    private void shootPreloads(ElapsedTime autoTimer) {
        if (!waitSeconds(SPIN_UP_SECONDS - autoTimer.seconds(), "Spin up flywheel")) {
            return;
        }
        intake.runForward();
        waitSeconds(FEED_SECONDS, "Feed POLLEN");
        intake.stop();
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
