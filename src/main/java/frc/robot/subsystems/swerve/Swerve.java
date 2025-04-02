package frc.robot.subsystems.swerve;

import frc.lib.math.GeometryUtils;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import frc.robot.commands.DriveToAprilTag;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
//import com.kauailabs.navx.ftc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder; 
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.subsystems.ArmStuff.Elevator;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.ArmStuff.Actuator2;


public class Swerve extends SubsystemBase {

    private final Field2d field2d = new Field2d();  // Represents the field for visualizing the robot's position
    private final SwerveDrivePoseEstimator m_poseEstimator;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods; // Array of swerve modules (wheels)
    public AHRS gyro = null; // Gyroscope to track robot's rotation

    // Relay channels for controlling LED lights on the robot (used for signaling, debugging, or indicator lights)
    public static Relay lightFr;
    public static Relay lightFl;
    public static Relay lightBr;
    public static Relay lightBl;

    // Zero the swerve modules to a known position (resetting their encoders or absolute position)
    public void zeroModules() {
        for (SwerveModule mod : mSwerveMods) {
            ((SwerveMod)mod).resetToAbsolute();  // Zero each swerve module
        }
    }

    // Constructor for the Swerve subsystem
    public Swerve() {

        // Set up Limelight camera pose relative to robot
        LimelightHelpers.setCameraPose_RobotSpace("", 
        0.47465,    // Forward offset (meters)
        -0.15,    // Side offset (meters)
        0.23495,    // Height offset (meters)
        0.0,    // Roll (degrees)
        6.64,   // Pitch (degrees)
        0.0     // Yaw (degrees)
        );

        // Initialize SmartDashboard to display field visualization
        SmartDashboard.putData("Field", field2d);

        // Initialize gyroscope (AHRS) for robot orientation
        gyro = new AHRS(NavXComType.kMXP_SPI);

        // Initialize relay channels for robot lights
        lightFr = new Relay(0);
        lightFl = new Relay(1);
        lightBr = new Relay(2);
        lightBl = new Relay(3);
        lightFl.set(Value.kOn);  // Turn on the front-left light

        // Initialize swerve modules for each corner of the robot (front-left, front-right, back-left, back-right)
        mSwerveMods = new SwerveModule[] {
            new SwerveMod(0, SwerveConstants.Swerve.Mod0.constants),
            new SwerveMod(1, SwerveConstants.Swerve.Mod1.constants),
            new SwerveMod(2, SwerveConstants.Swerve.Mod2.constants),
            new SwerveMod(3, SwerveConstants.Swerve.Mod3.constants)
        };

        // Initialize swerve odometry with gyro for position tracking
        swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, gyro.getRotation2d(), getModulePositions());

        // Zero the gyroscope to a known starting position
        zeroGyro();

        // Configure the robot's auto paths with PathPlanner
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();  // Load configuration from PathPlanner GUI
        } catch (Exception e) {
            // Handle exception if configuration loading fails
            e.printStackTrace();
            // Default configuration if PathPlanner configuration is missing
            config = new RobotConfig(74.088, 6.883, null);
        }

        // Configure PathPlanner for autonomous routines
        AutoBuilder.configure(
            this::getAprilOdom, // Robot pose supplier
            this::resetOdometry, // Reset odometry method for starting position in autonomous
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier for autonomous control
            (speeds, forwards) -> driveRobotRelative(speeds), // Method that will drive the robot with chassis speeds
            new PPHolonomicDriveController( // Holonomic drive controller with PID constants
                new PIDConstants(1.85, 0.0, 0.0), // Translation PID constants
                new PIDConstants(3.3, 0.0, 0.0) // Rotation PID constants
              ),
            config, // Robot configuration used in PathPlanner
            () -> {
              // Determine if the path should be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

        // Log PathPlanner configuration status
        SmartDashboard.putBoolean("Configured", AutoBuilder.isConfigured());

        // Initialize pose estimator for tracking robot position
        m_poseEstimator =
            new SwerveDrivePoseEstimator(
            SwerveConfig.swerveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(), // Front left
                mSwerveMods[1].getPosition(), // Front right
                mSwerveMods[2].getPosition(), // Back left
                mSwerveMods[3].getPosition() // Back right
            },
          new Pose2d(),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    }

    // Correct the chassis speeds for dynamics based on future pose
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;  // Loop time for prediction (20 ms)
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    // Get robot-relative speeds for autonomous (no joystick input needed)
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds(0, 0, 0); // Return 0 speeds (autonomous mode will update automatically)
    }

    // Update robot's odometry based on module positions and gyro data
    public void updateOdometry(){
        m_poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            mSwerveMods[0].getPosition(), // Front left
            mSwerveMods[1].getPosition(), // Front right
            mSwerveMods[2].getPosition(), // Back left
            mSwerveMods[3].getPosition()  // Back right
        });
        boolean doRejectUpdate = true;
        // Vision-based pose updates for improved localization
        LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 != null){
            if(!(Math.abs(gyro.getRate()) > 720) && mt2.tagCount > 0) // Reject vision updates if angular velocity is too high
            {
                doRejectUpdate = false;
            }
            else{
                doRejectUpdate = true;
            }

            if(!doRejectUpdate)
            {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,999999999));
                m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
            }
        }
    }

    // Drive robot with the specified chassis speeds relative to the field or robot
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(speeds);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], true); // Drive the swerve modules
        }
    }

    // Drive robot with translation and rotation inputs (field-relative or robot-relative)
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(),
        rotation,
        getYaw())
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);

        // Set the module states for each swerve module
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    

    public Pose2d getAprilOdom(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPosition(){
        return swerveOdometry.getPoseMeters(); // Ensure swerveOdometry is initialized properly
    }

    // Reset odometry to a specific pose
    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
    }

    // Get the states of the swerve modules (each wheel's speed and angle)
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    // Get the positions of the swerve modules (each wheel's position in space)
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    // Update the field visualization with the current robot pose
    public void updateField(Pose2d robotPose) {
        field2d.setRobotPose(robotPose);
    }

    // Zero the gyroscope at a specific angle (optional)
    public void zeroGyro(double deg) {
        if(SwerveConfig.invertGyro) {
            deg = -deg; // Invert if necessary
        }
        
        gyro.reset();
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    // Zero the gyroscope to 0 degrees
    public void zeroGyro() {  
       zeroGyro(0);
    }

    // Get the yaw (rotation) of the robot
    public Rotation2d getYaw() {
        return (SwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    // Set a low speed for testing purposes
    public void setSpeed3(){
        mSwerveMods[0].setSpeed2(0.2);
        mSwerveMods[1].setSpeed2(0.2);
        mSwerveMods[2].setSpeed2(0.2);
        mSwerveMods[3].setSpeed2(0.2);
    }

    // Auto-zero all swerve module encoders to a known state
    public void autoZeroWheels() {
        for (SwerveModule mod : mSwerveMods) {
            ((SwerveMod) mod).resetToAbsolute(); // Call resetToAbsolute for each module
        }
        System.out.println("Auto-zeroing wheels completed.");
    }

    // Periodic updates for the Swerve subsystem
    @Override
    public void periodic() {
        // Update odometry and robot position
        updateOdometry();
        Pose2d currentPose = getAprilOdom(); 
        updateField(currentPose);

        // Log current robot data to SmartDashboard for monitoring
        swerveOdometry.update(gyro.getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);   
        }
    }
}
