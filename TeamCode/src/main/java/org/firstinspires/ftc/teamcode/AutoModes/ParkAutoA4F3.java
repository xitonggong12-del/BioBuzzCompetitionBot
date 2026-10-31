package org.firstinspires.ftc.teamcode.AutoModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystem.Drivetrain;

/**
 * 按时间停车的自动：从 A4 离墙，再向左横移进入 A5 的 LOADING ZONE，拿 LEAVE + PARK。
 * Timed parking autonomous: leave the wall from A4, then strafe left into the A5 LOADING ZONE
 * for LEAVE + PARK.
 *
 * 起点：红方 A4（蓝方 F3），车尾贴联盟墙，车头朝场地中间，车身放在格子中间。
 * Start: red A4 (blue F3), back on the alliance wall, front facing the middle of the field,
 * robot centered in the tile.
 * 场地 180° 旋转对称，蓝方从 F3 跑同一个程序，向左横移进入 F2。
 * The field is 180° rotationally symmetric, so blue runs the same program from F3 and strafes left into F2.
 * 不用 Pinpoint、不射球；距离靠动力和时间，需要在场地上调。
 * Uses no Pinpoint and no shooting; distance comes from power and time and must be tuned on the field.
 */

@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAutoA4F3 extends LinearOpMode {

    // 先往前离墙：LOADING ZONE 只有约 11 in 深，只需离墙几英寸，车尾不能超出区域。
    // First drive forward off the wall: the LOADING ZONE is only ~11 in deep, so move just a few inches.
    private static final double FORWARD_POWER = 0.3;
    private static final double FORWARD_SECONDS = 0.3;

    // 再向左横移约 12 in，让车身约一半进入 A5；麦轮横移比直走慢，时间可能要比直走长。
    // Then strafe left ~12 in so about half the robot is in A5; mecanum strafing is slower than driving straight.
    private static final double STRAFE_POWER = 0.4;
    private static final double STRAFE_SECONDS = 1.0;

    // 两段动作之间停一下，避免惯性把前一段的动作带到下一段。
    // Pause between moves so momentum from one move does not carry into the next.
    private static final double PAUSE_SECONDS = 0.2;

    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Park Auto: red A4 / blue F3, back on wall, centered in tile");
        telemetry.addData("Forward", "%.2f power for %.1f s", FORWARD_POWER, FORWARD_SECONDS);
        telemetry.addData("Strafe left", "%.2f power for %.1f s", STRAFE_POWER, STRAFE_SECONDS);
        telemetry.update();

        waitForStart();

        // Drivetrain.drive 的 strafe 向右为正，所以向左横移用负值。
        // Drivetrain.drive strafe is positive to the right, so strafing left uses a negative value.
        driveFor("Leave wall", FORWARD_POWER, 0.0, FORWARD_SECONDS);
        driveFor("Pause", 0.0, 0.0, PAUSE_SECONDS);
        driveFor("Strafe into zone", 0.0, -STRAFE_POWER, STRAFE_SECONDS);
        drivetrain.stop();

        telemetry.addLine("Done");
        telemetry.update();
    }

    /**
     * 以固定动力开一段时间，到时间或 OpMode 停止后停车。
     * Drives at fixed power for a set time, then stops when the time is up or the OpMode ends.
     */
    private void driveFor(String step, double forward, double strafe, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drivetrain.drive(forward, strafe, 0.0);
            telemetry.addData(step, "%.1f / %.1f s", timer.seconds(), seconds);
            telemetry.update();
        }
        drivetrain.stop();
    }
}
