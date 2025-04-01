package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class AprilTag {
    double x, y, z;
    double zRotation, yRotation;
    
    public AprilTag(double x, double y, double z, double zRotation, double yRotation) {
        this.zRotation = zRotation - 180;
        this.yRotation = yRotation;
        this.x = ((x + (22 * Math.cos(zRotation*(3.141592653589793238462/180))))/ 39.37); // Convert inches to meters
        this.y = ((y + (22 * Math.sin(zRotation*(3.141592653589793238462/180))))/ 39.37);
        this.z = z / 39.37;
    }

    @Override
    public String toString() {
        return String.format("X: %.2f, Y: %.2f, Z: %.2f, Z-Rotation: %.1f°, Y-Rotation: %.1f°", 
                              x, y, z, zRotation, yRotation);
    }
}

public class AprilTagCoordinates {
    private static final Map<Integer, AprilTag> tagMap = new HashMap<>();

    static {
        tagMap.put(1, new AprilTag(657.37, 25.80, 58.50, 126, 0));
        tagMap.put(2, new AprilTag(657.37, 291.20, 58.50, 234, 0));
        tagMap.put(3, new AprilTag(455.15, 317.15, 51.25, 270, 0));
        tagMap.put(4, new AprilTag(365.20, 241.64, 73.54, 0, 30));
        tagMap.put(5, new AprilTag(365.20, 75.39, 73.54, 0, 30));
        tagMap.put(6, new AprilTag(530.49, 130.17, 12.13, 300, 0));
        tagMap.put(7, new AprilTag(546.87, 158.50, 12.13, 0, 0));
        tagMap.put(8, new AprilTag(530.49, 186.83, 12.13, 60, 0));
        tagMap.put(9, new AprilTag(497.77, 186.83, 12.13, 120, 0));
        tagMap.put(10, new AprilTag(481.39, 158.50, 12.13, 180, 0));
        tagMap.put(11, new AprilTag(497.77, 130.17, 12.13, 240, 0));
        tagMap.put(12, new AprilTag(33.51, 25.51, 58.5, 54, 0));
        tagMap.put(13, new AprilTag(33.51, 291.20, 58.50, 306, 0));
        tagMap.put(14, new AprilTag(325.68, 241.64, 73.54, 180, 0));
        tagMap.put(15, new AprilTag(325.68, 241.64, 73.54, 180, 0));
        tagMap.put(16, new AprilTag(235.73, -0.15, 51.25, 90, 0));
        tagMap.put(17, new AprilTag(160.39, 130.17, 12.13, 240, 0));
        tagMap.put(18, new AprilTag(144.00, 158.50, 12.13, 180, 0));
        tagMap.put(19, new AprilTag(160.39, 186.83, 12.13, 120, 0));
        tagMap.put(20, new AprilTag(193.10, 186.83, 12.13, 60, 0));
        tagMap.put(21, new AprilTag(209.49, 158.50, 12.13, 0, 0));
        tagMap.put(22, new AprilTag(193.10, 130.17, 12.13, 300, 0));
    }

    public static AprilTag getAprilTag(int id) {
        return tagMap.get(id);
    }

    public static Double getX(int tagId) {
        AprilTag tag = tagMap.get(tagId);
        return (tag != null) ? tag.x : null;
    }
    
    public static Double getY(int id) {
        AprilTag tag = tagMap.get(id);
        return (tag != null) ? tag.y : null;
    }
    
    public static Double getZ(int id) {
        AprilTag tag = tagMap.get(id);
        return (tag != null) ? tag.z : null;
    }

    public static Pose2d getPose2d(int tagId) {
    AprilTag tag = tagMap.get(tagId);
    if (tag != null) {
        return new Pose2d(tag.x, tag.y, Rotation2d.fromDegrees(tag.zRotation));
    }
    return null;
}
    public static void main(String[] args) {
        int testId = 1; // Change this to test different IDs
        AprilTag tag = getAprilTag(testId);
        if (tag != null) {
            System.out.println("AprilTag " + testId + ": " + tag);
        } else {
            System.out.println("AprilTag ID not found.");
        }
    }
}
