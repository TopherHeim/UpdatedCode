package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.States;
import frc.robot.Utility;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.commands.AprilTagCoordinates;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TeleopSwerve extends Command { 
  private final boolean m_leftSide = true; // Left or right side of the coral to go to
  private Command m_path;

  // Function to get the AprilTag ID detected by the Limelight
  public static int getAprilTagID() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tidEntry = table.getEntry("tid"); // 'tid' holds the detected AprilTag ID
    return (int) tidEntry.getDouble(-1); // Return the ID, or -1 if no tag is found
  }

  // Function to check if the AprilTag is in view and matches the current team color
  public static boolean coralTagInView() {
    return (Utility.aprilTagInView() &&
      (
        (Utility.teamColorIsRed() && Utility.aprilTagIDIsInList(Constants.AprilTags.coralRedTags)) ||
        (Utility.aprilTagIDIsInList(Constants.AprilTags.coralBlueTags))
      )
    );
  }

  // Finds the field position of the robot facing the AprilTag, lined up to the coral.
  // MATHS DESMOS: https://www.desmos.com/calculator/uagr4pd9gv
  public static Pose2d findGoalPos(Pose2d robotPos, Pose2d aprilTagPos, boolean leftSide) {
    double robotRot = robotPos.getRotation().getRadians();
    double faceTagAngle = robotRot - aprilTagPos.getRotation().getRadians(); // robotRot - tagRot, finds angle to face the AprilTag
    
    // Calculate the AprilTag's position on the field
    Translation2d tagFieldPos = new Translation2d(
      (aprilTagPos.getX() * Math.cos(robotRot)) + (aprilTagPos.getY() * Math.cos(robotRot - (Math.PI / 2))),
      (aprilTagPos.getX() * Math.sin(robotRot)) + (aprilTagPos.getY() * Math.sin(robotRot - (Math.PI / 2)))
    );

    double offsetHoriz = Constants.AprilTags.coralOffset.getX();
    double offsetOut = Constants.AprilTags.coralOffset.getY();

    if (leftSide) {
      offsetHoriz *= -1; // Flip to other side of the AprilTag
    }

    // Add offsets to find the position of the robot on the field next to the AprilTag
    Translation2d finalGoalPos = new Translation2d(
      tagFieldPos.getX() + (offsetHoriz * Math.cos(faceTagAngle - (Math.PI / 2))) + (offsetOut * Math.cos(faceTagAngle - Math.PI)),
      tagFieldPos.getY() + (offsetHoriz * Math.sin(faceTagAngle - (Math.PI / 2))) + (offsetOut * Math.sin(faceTagAngle - Math.PI))
    );

    return new Pose2d(finalGoalPos, Rotation2d.fromRadians(faceTagAngle));
  }   

  // Declare variables for Swerve drive and control inputs
  private Swerve s_Swerve;    
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier dampen;
  private DoubleSupplier speedDial;
  private BooleanSupplier zero;
  private BooleanSupplier integratedZero;

  // Declare PID controller for rotation
  private PIDController rotationController;

  // Constructor to initialize the TeleopSwerve command
  public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier speedDial, BooleanSupplier zero, BooleanSupplier integratedZero) {
    this.s_Swerve = s_Swerve;
    this.zero = zero;
    addRequirements(s_Swerve); // Add the swerve subsystem to the command requirements

    // Initialize PID controller for rotation
    rotationController = new PIDController(0.01, 0, 0 );
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(3);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.dampen = dampen;
    this.speedDial = speedDial;
  }

  // This method is called periodically while the command is scheduled
  @Override
  public void execute() {
    // Check if the robot should zero its position
    if(zero.getAsBoolean() == true){
      // Get the robot's current position from the Limelight
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (!(mt2.tagCount == 0)) {
        int TagId = getAprilTagID();

        // Get the robot's current pose and the goal pose based on the detected AprilTag
        Pose2d currentRobotPose = s_Swerve.getAprilOdom();  
        Pose2d goalPos = AprilTagCoordinates.getPose2d(TagId);
        
        // Create waypoints for the path from current position to goal position
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentRobotPose, goalPos);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            Constants.AprilTags.constraints,
            null,
            new GoalEndState(0, goalPos.getRotation())
        );

        path.preventFlipping = true; // Prevent path flipping
        m_path = AutoBuilder.followPath(path); // Build the path following command
        
        // Initialize the path command and schedule it
        if (m_path != null) {
            m_path.schedule();
            System.out.println("Path initialized!");
        }
      }
    }

    /* Get Values, Apply Deadband, and Adjust Speed */
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);

    // Handle different drive states for heading control
    switch(States.driveState){
        case d0:
            rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(0)); // Heading lock to 0 degrees
            break;
        case d90:
            rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(90)); // Heading lock to 90 degrees
            break;
        case d180:
            rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(180)); // Heading lock to 180 degrees
            break;
        case d270:
            rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(270)); // Heading lock to 270 degrees
            break;
        case standard:
            rotationVal = rotationVal * SwerveConfig.maxAngularVelocity; // Normal control for rotation
            break;
    }

    // Drive the robot using the calculated values
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(SwerveConfig.maxSpeed), // Apply translation speed
        rotationVal, // Apply rotation speed
        !robotCentricSup.getAsBoolean(), // Use field-centric control if true
        true // Enable swerve drive field-relative mode
    );
  }
}
