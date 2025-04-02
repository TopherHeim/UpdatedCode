
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import javax.sound.midi.MidiDevice;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;


/** Swerve Module class utilizing REV Robotics motor controllers and CTRE CANcoder absolute encoders. **/
public class SwerveMod implements SwerveModule
{
    public int moduleNumber;  // Module ID for this swerve modul
    private Rotation2d angleOffset;  // Offset for angle calibration to maintain consistent module orientation

    // Motor controllers
    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;
    private SparkClosedLoopController controller;

    private SparkMaxConfig motorConfig;  // Motor configuration object
    private SparkClosedLoopController closedLoopController;

    // Encoders for position and velocity feedback
    private CANcoder angleEncoder;
    private RelativeEncoder encoder;
    private RelativeEncoder relDriveEncoder;

    // Constructor for initializing the Swerve module with its parameters and configuring motors/encoders
    public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Motor Configuration */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless); // Initialize angle motor
        configAngleMotor();  // Configure the angle motor settings
        
        /* Drive Motor Configuration */
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); // Initialize drive motor
        configDriveMotor();  // Configure the drive motor settings

        /* Angle Encoder Configuration */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);  // Initialize absolute encoder (CTRE CANcoder)
        configEncoders();  // Configure the encoders for both angle and drive motors
    }

    // Configures the encoders for relative and absolute position feedback
    private void configEncoders()
    {     
        // Configuration for relative drive encoder
        SparkMaxConfig configRelDrive = new SparkMaxConfig();
        relDriveEncoder = mDriveMotor.getEncoder();  // Initialize the drive motor encoder
        relDriveEncoder.setPosition(0);  // Reset encoder position to zero

        encoder = mAngleMotor.getEncoder();  // Initialize the angle motor encoder

        // Apply CANcoder configuration (for absolute encoder)
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new SwerveConfig().canCoderConfig);

        // Set position and velocity conversion factors for drive encoder
        configRelDrive.encoder
            .positionConversionFactor(SwerveConfig.driveRevToMeters)  // Convert revolutions to meters
            .velocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);  // Convert RPM to m/s for velocity

        // Set position and velocity conversion factors for angle encoder
        motorConfig.encoder
            .positionConversionFactor(SwerveConfig.DegreesPerTurnRotation)  // Convert to degrees
            .velocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);  // Convert to degrees per second (RPM)

        // Apply configurations to the motors
        mDriveMotor.configure(configRelDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mAngleMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        resetToAbsolute();  // Reset the encoder to its absolute position based on the CANcoder
    }

    // Resets the module's encoders to the absolute position from the CANcoder, accounting for offsets
    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees();  // Get absolute position from CANcoder
        double zeroedPosition = absolutePosition - angleOffset.getDegrees();  // Apply offset for calibration
        
        // Adjust zeroed position if necessary to ensure it falls within valid range
        if (zeroedPosition > 180) {
            zeroedPosition -= 360;  // Normalize to the range (-180 to +180)
        } 

        encoder.setPosition(zeroedPosition);  // Set motor encoder to zeroed position
        // Debugging output to confirm reset
        System.out.println("Module " + moduleNumber + ": Reset to Absolute -> CANCoder = " + absolutePosition 
                        + ", Offset = " + angleOffset.getDegrees() 
                        + ", Zeroed Position = " + zeroedPosition + ", Actual Position  = " + encoder.getPosition());
    }

   // Configures the angle motor for the swerve module
    private void configAngleMotor()
    {
        // Obtain the closed-loop controller for the angle motor
        closedLoopController = mAngleMotor.getClosedLoopController();

        // Initialize the SparkMax motor configuration object
        motorConfig = new SparkMaxConfig();

        // Set the configuration for the angle motor: inversion, idle mode, and current limit
        motorConfig
            .inverted(SwerveConfig.angleMotorInvert)  // Whether the motor is inverted (clockwise or counterclockwise)
            .idleMode(SwerveConfig.angleIdleMode)  // Idle mode of the motor when not actively controlled
            .smartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);  // Set the maximum continuous current the motor can draw

        // Configure closed-loop control for the motor with PID settings and feedforward
        motorConfig.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)  // Uncomment to use encoder feedback
            .pid(SwerveConfig.angleKP, SwerveConfig.angleKI, SwerveConfig.angleKD)  // PID coefficients for angle control
            .velocityFF(SwerveConfig.angleKF)  // Feedforward term for velocity control
            .outputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);  // Set the output range for the motor (-1 to 1)

        // Apply the motor configuration to the angle motor
        mAngleMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Configures the drive motor for the swerve module
    private void configDriveMotor()
    {
        // Initialize a new SparkMax motor configuration object for the drive motor
        SparkMaxConfig configDrive = new SparkMaxConfig();

        // Set the configuration for the drive motor: inversion, idle mode, and current limit
        configDrive
            .inverted(SwerveConfig.driveMotorInvert)  // Whether the drive motor is inverted
            .idleMode(SwerveConfig.driveIdleMode)  // Idle mode of the drive motor
            .smartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);  // Set the maximum continuous current the motor can draw

        // Configure closed-loop control for the drive motor with PID settings and feedforward
        configDrive.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)  // Uncomment to use encoder feedback
            .velocityFF(SwerveConfig.driveKF)  // Feedforward term for drive velocity control
            .pid(SwerveConfig.driveKP, SwerveConfig.driveKI, SwerveConfig.driveKD)  // PID coefficients for speed control
            .outputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);  // Set the output range for the drive motor (-1 to 1)

        // Apply the motor configuration to the drive motor
        mDriveMotor.configure(configDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Sets the desired state for the swerve module (desired speed and angle)
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        // Optimize the desired state by adjusting the angle for minimal rotation needed
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        // Set the desired angle of the swerve module
        setAngle(desiredState);

        // Set the desired speed of the swerve module
        setSpeed(desiredState, isOpenLoop);
    }

    // Sets the speed of the swerve module's drive motor (open-loop or closed-loop control)
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        if (isOpenLoop)
        {
            // In open-loop, set the motor speed as a percentage of the maximum speed
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);  // Set the speed of the drive motor as a percentage
            return;
        }

        // In closed-loop, set the motor speed using the PID controller
        double velocity = desiredState.speedMetersPerSecond;

        // Get the closed-loop controller for the drive motor
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();

        // Set the desired velocity as a reference for closed-loop control
        controller.setReference(velocity, ControlType.kVelocity);
        // Keep It Up :)
    }
    
    /* Deleting this killed the code */
    public void setSpeed2(double s){
        //suck my ballz

    }
    
    // Sets the angle of the swerve module's angle motor based on the desired state
    private void setAngle(SwerveModuleState desiredState)
    {
        // If the speed is very low (within a 1% threshold), stop the motor to avoid jittering
        if (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01))  // Dead zone for small inputs
        {
            mAngleMotor.stopMotor();  // Stop the angle motor
            return;
        }

        // Get the desired angle for the module
        Rotation2d angle = desiredState.angle;

        // Prevent rotating the module if speed is below the threshold to avoid unnecessary movement
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();

        // Get the angle reference in degrees for the controller
        double degReference = angle.getDegrees();

        // Set the desired angle as a reference for closed-loop position control
        controller.setReference(degReference, ControlType.kPosition);
    }

    // Returns the current angle of the module (from the encoder)
    private Rotation2d getAngle()
    {
        // Return the angle based on the encoder's current position
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    // Returns the current absolute angle of the module from the CANcoder
    public Rotation2d getCanCoder()
    {
        // Get the absolute position from the CANcoder and return it as a Rotation2d object
        return Rotation2d.fromDegrees((angleEncoder.getAbsolutePosition().getValue().in(Degrees))); // Returns the absolute angle in degrees
    }

    // Getter for the module number (ID)
    public int getModuleNumber()
    {
        return moduleNumber;  // Return the module's unique ID
    }

    // Setter for the module number (ID)
    public void setModuleNumber(int moduleNumber) 
    {
        this.moduleNumber = moduleNumber;
    }
  

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
            relDriveEncoder.getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            relDriveEncoder.getPosition(), 
            getAngle()
        );
    }
}
