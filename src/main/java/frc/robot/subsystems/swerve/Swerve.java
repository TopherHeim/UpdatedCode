package frc.robot.subsystems.swerve;

import frc.lib.math.GeometryUtils;
import frc.robot.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.text.BreakIterator;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;




public class Swerve extends SubsystemBase {


    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro = null;
    public static Relay lightFr;
    public static Relay lightFl;
    public static Relay lightBr;
    public static Relay lightBl;

    public void zeroModules() {
    for (SwerveModule mod : mSwerveMods) {
        ((SwerveMod)mod).resetToAbsolute();  // Zero each swerve module
    }
}

    public Swerve() {
        
        gyro = new AHRS(NavXComType.kMXP_SPI); // (May need to change this: NavXUpdateRate.k200Hz) 
        //gyro.configFactoryDefault();
        lightFr = new Relay(0);
        lightFl = new Relay(1);
        lightBr = new Relay(2);
        lightBl = new Relay(3);
        lightFl.set(Value.kOn);

        
     

        mSwerveMods = new SwerveModule[] {
           
            new SwerveMod(0, SwerveConstants.Swerve.Mod0.constants),
            new SwerveMod(1, SwerveConstants.Swerve.Mod1.constants),
            new SwerveMod(2, SwerveConstants.Swerve.Mod2.constants),
            new SwerveMod(3, SwerveConstants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, gyro.getRotation2d(), getModulePositions());
        zeroGyro();
        /*Good Job =D */
        RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();

      // Default, if not setup within Pathplanner's settings already
            config = new RobotConfig(74.088, 6.883, null);
    }

        AutoBuilder.configure(
            this::getPosition, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, forwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(1.7, 0, 0.0), // Translation PID constants
                new PIDConstants(2.2, 0.0, 0.0) // Rotation PID constants
              ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
        SmartDashboard.putBoolean("Configured", AutoBuilder.isConfigured());
    }
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
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
    public ChassisSpeeds getRobotRelativeSpeeds() {
        // PathPlanner will provide the ChassisSpeeds for autonomous driving
        return new ChassisSpeeds(0, 0, 0); // This will be updated automatically in autonomous, no joystick input needed
    }
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Convert ChassisSpeeds to your robot's swerve module states
        SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(speeds);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], true); // Drive the swerve modules
        }
    }

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
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    
    public Pose2d getPosition(){
        return swerveOdometry.getPoseMeters(); // Ensure swerveOdometry is initialized properly
    }
    public Pose2d getPose() {
        Pose2d p =  swerveOdometry.getPoseMeters();
        return new Pose2d(-p.getX(),-p.getY(),  p.getRotation());
    }
    

    public void resetOdometry(Pose2d pose) {
        
        swerveOdometry.resetPosition(
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
       
    }
    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(double deg) {
        if(SwerveConfig.invertGyro) {
            deg = -deg;
        }
        
        gyro.reset();
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    public void zeroGyro() {  
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (SwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void setSpeed3(){
        mSwerveMods[0].setSpeed2(0.2);
        mSwerveMods[1].setSpeed2(0.2);
        mSwerveMods[2].setSpeed2(0.2);
        mSwerveMods[3].setSpeed2(0.2);
    }
    public void autoZeroWheels() {
        for (SwerveModule mod : mSwerveMods) {
            ((SwerveMod) mod).resetToAbsolute(); // Call resetToAbsolute for each module
        }
        System.out.println("Auto-zeroing wheels completed.");
    }
    /* 
    public void autonDrive(double s){
        if (mSwerveMods[0].getModuleNumber() == 0){
            mSwerveMods[0].setSpeed2(s);
        }
        if (mSwerveMods[1].getModuleNumber() == 1){
            mSwerveMods[1].setSpeed2(s*-1);
        }
        if (mSwerveMods[2].getModuleNumber() == 2){
            mSwerveMods[2].setSpeed2(s);
        }
        if (mSwerveMods[3].getModuleNumber() == 3){
            mSwerveMods[3].setSpeed2(s*-1);
        }

    }
    */
    @Override
    public void periodic() {
        
        swerveOdometry.update(gyro.getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);   
            //
        }
    }
}
