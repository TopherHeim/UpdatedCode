package frc;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.net.URL;

public class EagleEyeView {

    private static final double FIELD_WIDTH = 16.5;  // Field width in feet (example)
    private static final double FIELD_LENGTH = 8.0;  // Field length in feet (example)

    // Default robot position for testing (in feet)
    private static double robotX = 5.0;
    private static double robotY = 3.0;
    private static double robotAngle = 90.0;

    // Method to set up the Eagle Eye view on Shuffleboard
    public static void setupEagleEyeView() throws IOException {
        // Load the field image (top-down view of the field)
        URL fieldImageUrl = new URL("https://example.com/path_to_field_image.png"); // Replace with actual field image URL
        BufferedImage fieldImage = ImageIO.read(fieldImageUrl);

        // Set up Shuffleboard Tab for the Eagle Eye view
        ShuffleboardTab tab = Shuffleboard.getTab("EagleEyeView");
        tab.add("Field", new JLabel(new ImageIcon(fieldImage))).withWidget(BuiltInWidgets.kField);

        // Create a custom panel to draw the robot's position
        JPanel panel = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                super.paintComponent(g);

                // Draw the field image (scaled to fit the panel)
                g.drawImage(fieldImage, 0, 0, getWidth(), getHeight(), null);

                // Draw the robot's position
                g.setColor(Color.RED);
                // Scale robot position to match the field dimensions
                int robotXPos = (int) ((robotX / FIELD_WIDTH) * getWidth());
                int robotYPos = (int) ((robotY / FIELD_LENGTH) * getHeight());

                // Draw a circle for the robot's position
                g.fillOval(robotXPos - 10, robotYPos - 10, 20, 20);

                // Optionally, draw a line for the robot's orientation (facing direction)
                int robotEndX = (int) (robotXPos + Math.cos(Math.toRadians(robotAngle)) * 30);  // Adjust line length
                int robotEndY = (int) (robotYPos + Math.sin(Math.toRadians(robotAngle)) * 30);
                g.setColor(Color.BLUE);
                g.drawLine(robotXPos, robotYPos, robotEndX, robotEndY);
            }
        };

        // Add the custom panel to Shuffleboard
        tab.add("Eagle Eye View", panel).withWidget(BuiltInWidgets.kGraph);
    }

    // Method to update the robot's position (example)
    public static void updateRobotPosition(double x, double y, double angle) {
        robotX = x;
        robotY = y;
        robotAngle = angle;
    }
}
