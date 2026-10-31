package org.firstinspires.ftc.teamcode.AutoModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystem.Drivetrain;

/**
 * 最简单的自动：按时间往前开一小段，离开墙拿 LEAVE 分，然后停车。
 * Simplest autonomous: drive forward for a set time to leave the wall for LEAVE, then stop.
 *
 * 不用 Pinpoint，也不射球；起点任意，只要车尾贴墙、车头朝场地中间、前方没有障碍。
 * Uses no Pinpoint and no shooting; any start works as long as the back is on the wall,
 * the front faces the middle of the field, and the path ahead is clear.
 */
@Autonomous(name = "Leave Auto", group = "Auto")
public class LeaveAuto extends LinearOpMode {

    // 动力和时间决定开多远；先低速试，再按实际距离调整。
    // Power and time set how far the robot drives; start slow, then tune from the measured distance.
    private static final double DRIVE_POWER = 0.3;
    private static final double DRIVE_SECONDS = 1.0;

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Leave Auto: back on wall, path ahead clear");
        telemetry.addData("Drive", "%.2f power for %.1f s", DRIVE_POWER, DRIVE_SECONDS);
        telemetry.update();

        waitForStart();

        // 开始后往前开，到时间或比赛停止就停下。
        // After START, drive forward until the time is up or the OpMode stops.
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
            drivetrain.drive(DRIVE_POWER, 0.0, 0.0);
            telemetry.addData("Driving", "%.1f / %.1f s", timer.seconds(), DRIVE_SECONDS);
            telemetry.update();
        }
        drivetrain.stop();

        telemetry.addLine("Done");
        telemetry.update();
    }
}
