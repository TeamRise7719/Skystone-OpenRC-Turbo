package org.firstinspires.ftc.teamcode.Fraser.Subsystems.OpenCV;

public interface VisionSystem {
    enum Type {VUFORIA, OPENCV}
    enum TargetType {SKYSTONE, BRIDGE, PERIMETER, NONE_JUST_RUN_FOREVER}

    void startLook(TargetType targetType);
    void stopLook();
    boolean found();
}