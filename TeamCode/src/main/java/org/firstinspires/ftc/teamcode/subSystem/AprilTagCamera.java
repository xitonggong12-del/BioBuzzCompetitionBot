package org.firstinspires.ftc.teamcode.subSystem;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * 用 USB 摄像头识别 BioBuzz 场地的 AprilTag。
 * Detects the BioBuzz field AprilTags with a USB webcam.
 *
 * BioBuzz 的标签是 4 个一组的“簇”，SDK 把一整簇当成一个目标返回位姿：
 * BioBuzz tags come in clusters of 4, and the SDK returns one pose for the whole cluster:
 *   RED SCORING 30-33 | RED AUDIENCE 34-37 | BLUE AUDIENCE 38-41 | BLUE SCORING 42-45
 *
 * 距离单位英寸，角度单位度；bearing 为正表示目标在摄像头左边。
 * Distances are in inches and angles in degrees; positive bearing means the target is left of the camera.
 */
public class AprilTagCamera {

    // 名称 "Webcam 1" 必须和 Driver Station 中的机器人配置一致，类型选 Webcam。
    // The name "Webcam 1" must match the robot configuration on Driver Station, configured as a Webcam.
    private static final String DEVICE_NAME = "Webcam 1";

    // 分辨率越高识别越远，但帧率越低；640x480 自带标定数据，一般够用。
    // Higher resolution sees farther but lowers frame rate; 640x480 has built-in calibration and is usually enough.
    private static final Size RESOLUTION = new Size(640, 480);

    // 簇名称，和 SDK 标签库里的一致。 / Cluster names, matching the SDK tag library.
    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    public AprilTagCamera(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, DEVICE_NAME))
                .setCameraResolution(RESOLUTION)
                .addProcessor(aprilTag)
                .build();
    }

    /** 当前帧里所有识别到的目标（簇或单个标签）。 / All targets (clusters or single tags) in the latest frame. */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * 找指定名称的簇（如 RED_SCORING），没看到返回 null。
     * Finds the cluster with the given name (e.g. RED_SCORING), or null if not seen.
     */
    public AprilTagClusterDetection getCluster(String name) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata != null && name.equals(cluster.metadata.name)) {
                    return cluster;
                }
            }
        }
        return null;
    }

    /** 目标的显示名称，供 Telemetry 用。 / Display name of a target, for Telemetry. */
    public static String describe(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return String.format("%s (%d%% seen)", cluster.metadata.name, cluster.percentClusterFound);
        }
        AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
        return single.metadata != null
                ? String.format("ID %d %s", single.id, single.metadata.name)
                : String.format("ID %d Unknown", single.id);
    }

    /** 摄像头每秒帧数，供 Telemetry 显示。 / Camera frames per second, for Telemetry. */
    public float getFps() {
        return visionPortal.getFps();
    }

    /** OpMode 结束时关闭摄像头。 / Closes the camera when the OpMode ends. */
    public void close() {
        visionPortal.close();
    }
}
