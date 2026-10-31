package org.firstinspires.ftc.teamcode.AopModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystem.AprilTagCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * 只测试 AprilTag 摄像头，只需要在 Driver Station 配置 "Webcam 1" 一个设备。
 * Tests only the AprilTag camera; only the "Webcam 1" device needs to be configured on Driver Station.
 *
 * 画面：INIT 后在 Driver Station 右上角菜单选 "Camera Stream" 查看。
 * Video: after INIT, open "Camera Stream" from the Driver Station top-right menu.
 */
@TeleOp(name = "AprilTag Test", group = "test1")
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        AprilTagCamera camera = new AprilTagCamera(hardwareMap);

        // INIT 阶段也显示识别结果，方便对准摄像头。
        // Show detections during INIT too, to help aim the camera.
        while (opModeInInit()) {
            showDetections(camera);
        }

        while (opModeIsActive()) {
            showDetections(camera);
        }

        // OpMode 结束时关闭摄像头。
        // Close the camera when the OpMode ends.
        camera.close();
    }

    private void showDetections(AprilTagCamera camera) {
        List<AprilTagDetection> detections = camera.getDetections();
        telemetry.addData("FPS", "%.1f", camera.getFps());
        telemetry.addData("Targets", detections.size());

        for (AprilTagDetection detection : detections) {
            telemetry.addLine(AprilTagCamera.describe(detection));
            // 不在标签库里的标签没有位姿。 / Tags not in the library have no pose.
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format(
                        "  Range %.1f in | Bearing %.1f° | Yaw %.1f°",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.yaw
                ));
                telemetry.addLine(String.format(
                        "  X %.1f | Y %.1f | Z %.1f in",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                ));
            }
        }
        telemetry.update();
    }
}
