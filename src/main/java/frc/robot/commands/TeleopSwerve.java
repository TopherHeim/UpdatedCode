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

  public static int getAprilTagID() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tidEntry = table.getEntry("tid"); // 'tid' holds the detected AprilTag ID
    return (int) tidEntry.getDouble(-1); // Return the ID, or -1 if no tag is found
}
  // Checking for the AprilTag ID corresponding to the current team color
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
      // X = (tagX * cos(robotRot)) + (tagY * cos(robotRot - pi/2))
      (aprilTagPos.getX() * Math.cos(robotRot))  +  (aprilTagPos.getY() * Math.cos(robotRot - (Math.PI / 2))),
      (aprilTagPos.getX() * Math.sin(robotRot))  +  (aprilTagPos.getY() * Math.sin(robotRot - (Math.PI / 2)))
    );

    double offsetHoriz = Constants.AprilTags.coralOffset.getX();
    double offsetOut = Constants.AprilTags.coralOffset.getY();

    if (leftSide) {
      offsetHoriz *= -1; // Flip to other side of the AprilTag
    }

    // Add offsets to find the position of the robot on the field next to the AprilTag
    Translation2d finalGoalPos = new Translation2d(
      // X = tagFieldX + (offsetX * Math.cos(faceAngle - PI/2)) + (offsetY * Math.cos(faceAngle - PI/2))
      tagFieldPos.getX()  +  (offsetHoriz * Math.cos(faceTagAngle - (Math.PI / 2)))  +  (offsetOut * Math.cos(faceTagAngle - Math.PI)),
      tagFieldPos.getY()  +  (offsetHoriz * Math.sin(faceTagAngle - (Math.PI / 2)))  +  (offsetOut * Math.sin(faceTagAngle - Math.PI))
    );

    return new Pose2d(finalGoalPos, Rotation2d.fromRadians(faceTagAngle));
  }   
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier dampen;
    private DoubleSupplier speedDial;
    private BooleanSupplier zero;
    private BooleanSupplier integratedZero;

    private PIDController rotationController;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier dampen, DoubleSupplier speedDial, BooleanSupplier zero, BooleanSupplier integratedZero) {
        this.s_Swerve = s_Swerve;
        this.zero = zero;
        addRequirements(s_Swerve);

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
//This boi Complex
    @Override
    public void execute() {
        if(zero.getAsBoolean() == true){
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
          if (!(mt2.tagCount == 0)) {
            int TagId = getAprilTagID();
  
              Pose2d currentRobotPose = s_Swerve.getAprilOdom();  
              // Calculate goal pose
              Pose2d goalPos = AprilTagCoordinates.getPose2d(TagId);
              // Create path from current robot position to the new position
              List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentRobotPose, goalPos);
            System.out.println(goalPos);
              PathPlannerPath path = new PathPlannerPath(
                  waypoints,
                  Constants.AprilTags.constraints,
                  null,
                  new GoalEndState(0, goalPos.getRotation())
              );
  
              path.preventFlipping = true;
              m_path = AutoBuilder.followPath(path);
              
              // Initialize the path command
              if (m_path != null) {
                  m_path.schedule();
                  System.out.println("Path initialized!");
              }
      } 
  }
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);
        double  rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * (dampen.getAsBoolean() ? 0.2 : 1) * ((speedDial.getAsDouble() + 1) / 2);

        //heading direction state
        switch(States.driveState){
            case d0:

                //heading lock
               rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(0));
                break;
            case d90:

                //heading lock
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(90));
                break;
            case d180:

                //heading lock
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(180));
                break;
            case d270:

                //heading lock
                rotationVal = rotationController.calculate(s_Swerve.getYaw().getRadians(), Units.degreesToRadians(270));
                break;


            case standard:
            
                //normal
                rotationVal = rotationVal * SwerveConfig.maxAngularVelocity;
                break;
        }
       
         



        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConfig.maxSpeed), 
            rotationVal,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}