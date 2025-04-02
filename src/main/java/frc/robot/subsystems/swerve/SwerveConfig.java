package frc.robot.subsystems.swerve;

// Import necessary classes for the CANcoder, sensor signals, and motor control
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SwerveConfig {

    // CANcoder configuration object for angle sensor
    public CANcoderConfiguration canCoderConfig;

    // Idle modes for drive and angle motors
    public static final IdleMode driveIdleMode = IdleMode.kBrake; // Drive motor will brake when idle
    public static final IdleMode angleIdleMode = IdleMode.kBrake; // Angle motor will brake when idle

    // Motor power constants for drive and angle motors
    public static final double drivePower = 1;  // Full power for drive motors
    public static final double anglePower = .9; // Reduced power for angle motors

    // Gyro inversion setting
    public static final boolean invertGyro = true; // Ensure Gyro is CCW+ CW-

    // Selecting the swerve module configuration (choose a specific module from COTS)
    public static final COTSNeoSwerveConstants chosenModule =  
        COTSNeoSwerveConstants.SDSMK4(COTSNeoSwerveConstants.driveGearRatios.SDSMK4_L2);

    /* Drivetrain Constants */
    // Track width and wheelbase of the robot in meters (converted from inches)
    public static final double trackWidth = Units.inchesToMeters(18.0); 
    public static final double wheelBase = Units.inchesToMeters(20.0); 
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics - Defining the swerve module positions relative to the robot */
    // These positions represent the distance from the robot's center to each module in X and Y directions.
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),  // Front-left module
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Front-right module
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Back-left module
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // Back-right module
    );

    /* Module Gear Ratios */
    // Gear ratios for drive and angle motors as provided by the selected module
    public static final double driveGearRatio = chosenModule.driveGearRatio; // Drive gear ratio from selected module
    public static final double angleGearRatio = chosenModule.angleGearRatio; // Angle gear ratio from selected module

    /* Encoder setup for drive and angle motors */
    // Converting revolutions to meters and RPM to meters per second for drive motors
    public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio); // Meters per drive revolution
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60; // Meters per second for drive RPM
    // Conversion of a single turn of the angle motor to degrees
    public static final double DegreesPerTurnRotation = 360 / angleGearRatio; // Degrees per turn for the angle motor

    /* Motor Inversions - Configuring whether motors are inverted or not */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert; // Angle motor inversion
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert; // Drive motor inversion

    /* Angle Encoder Inversion */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert; // Whether the angle encoder is inverted

    /* Swerve Current Limiting */
    // Current limiting values for angle and drive motors (to protect the motors from overcurrent)
    public static final int angleContinuousCurrentLimit = 20; // Continuous current limit for angle motors
    public static final int anglePeakCurrentLimit = 40; // Peak current limit for angle motors
    public static final double anglePeakCurrentDuration = 0.1; // Duration of peak current for angle motors
    public static final boolean angleEnableCurrentLimit = true; // Enable current limiting for angle motors

    public static final int driveContinuousCurrentLimit = 35; // Continuous current limit for drive motors
    public static final int drivePeakCurrentLimit = 60; // Peak current limit for drive motors
    public static final double drivePeakCurrentDuration = 0.1; // Duration of peak current for drive motors
    public static final boolean driveEnableCurrentLimit = true; // Enable current limiting for drive motors

    /* Ramp Rates - Smooth acceleration to prevent sudden jerks */
    public static final double openLoopRamp = 0.25; // Ramp rate for open-loop control (helps with control and system stability)
    public static final double closedLoopRamp = 0.0; // Ramp rate for closed-loop control (set to 0 for instant response)

    /* PID values for angle motors */
    // PID controller constants for the angle motors (adjust based on desired behavior)
    public static final double angleKP = 0.05; // Proportional gain for angle motor PID
    public static final double angleKI = 0; // Integral gain for angle motor PID
    public static final double angleKD = 0; // Derivative gain for angle motor PID
    public static final double angleKF = 0; // Feedforward gain for angle motor PID

    /* PID values for drive motors */
    // PID controller constants for the drive motors (adjust based on desired behavior)
    public static final double driveKP = 0.1;  // Proportional gain for drive motor PID
    public static final double driveKI = 0.0;  // Integral gain for drive motor PID
    public static final double driveKD = 0.0;  // Derivative gain for drive motor PID
    public static final double driveKF = 0.0;  // Feedforward gain for drive motor PID

    /* Characterization values for the drive motors used in system identification (SYSID) */
    // These values help model the drive system for better control and optimization
    public static final double driveKS = (0.32); // Static friction coefficient (volts)
    public static final double driveKV = (1.51); // Velocity gain (volts / m/s)
    public static final double driveKA = (0.27); // Acceleration gain (volts / m/s^2)

    /* Swerve Profiling Values - Max speeds and angular velocities for robot movement */
    public static final double maxSpeed = 5.0; // Maximum speed of the robot in meters per second
    public static final double maxAngularVelocity = 5.0; // Maximum angular velocity in radians per second

    /* Constructor for SwerveConfig - Initializes CANcoder configuration */
    public SwerveConfig() {
        canCoderConfig = new CANcoderConfiguration();
        // Configuring the CANcoder sensor settings
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Discontinuity point for absolute sensor (0 to 360 degrees)
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // Sensor direction (counterclockwise is positive)
        
    }
}
