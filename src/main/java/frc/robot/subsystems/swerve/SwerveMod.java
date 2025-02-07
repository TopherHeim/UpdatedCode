
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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;


/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveMod implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;
   // private Rotation2d lastAngle;

    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;
    private SparkClosedLoopController controller;

    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;

    private CANcoder angleEncoder;
    private RelativeEncoder encoder;
    private RelativeEncoder relDriveEncoder;

    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();
        //mAngleMotor.setInverted(true);
        /* Drive Motor Config */
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

         /* Angle Encoder Config */
    
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();


       // lastAngle = getState().angle;
    }


    private void configEncoders()
    {     
       // absolute encoder   
        SparkMaxConfig configRelDrive = new SparkMaxConfig();

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        encoder = mAngleMotor.getEncoder();

        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new SwerveConfig().canCoderConfig);

        
        configRelDrive.encoder
            .positionConversionFactor(SwerveConfig.driveRevToMeters)
            .velocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);

        motorConfig.encoder
            .positionConversionFactor(SwerveConfig.DegreesPerTurnRotation)
            .velocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);


        mDriveMotor.configure(configRelDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        mAngleMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        resetToAbsolute();
    }
    public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees();  // Get CANCoder absolute position
    double zeroedPosition = absolutePosition - angleOffset.getDegrees();  // Apply offset
    // Adjust zeroed position to match the range of the relative encoder (-180 to +180)
    if (zeroedPosition > 180) {
        zeroedPosition -= 360;
    } 

    encoder.setPosition(zeroedPosition);  // Reset motor encoder position

    System.out.println("Module " + moduleNumber + ": Reset to Absolute -> CANCoder = " + absolutePosition 
                        + ", Offset = " + angleOffset.getDegrees() 
                        + ", Zeroed Position = " + zeroedPosition + ", Actual Position  = " + encoder.getPosition());
}

    private void configAngleMotor()
    {
        closedLoopController = mAngleMotor.getClosedLoopController();
        motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(SwerveConfig.angleMotorInvert)
            .idleMode(SwerveConfig.angleIdleMode)
            .smartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConfig.angleKP, SwerveConfig.angleKI, SwerveConfig.angleKD)
            .velocityFF(SwerveConfig.angleKF)
            .outputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
        mAngleMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
       
    }

    private void configDriveMotor()
    {        
        SparkMaxConfig configDrive = new SparkMaxConfig();

        configDrive
            .inverted(SwerveConfig.driveMotorInvert)
            .idleMode(SwerveConfig.driveIdleMode)
            .smartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
        configDrive.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .velocityFF(SwerveConfig.driveKF)
            .pid(SwerveConfig.driveKP, SwerveConfig.driveKI, SwerveConfig.driveKD)
            .outputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);
        mDriveMotor.configure(configDrive, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
       
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        
        
        // CTREModuleState functions for any motor type.
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        /* 
        if(mDriveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
         */
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setReference(velocity, ControlType.kVelocity);
     // Keep It Up :)   
    }
    
    public void setSpeed2(double s){
        //suck my ballz

    }
    
    private void setAngle(SwerveModuleState desiredState)
    {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed *0.01)) //dead zone for small inputs
        {
            mAngleMotor.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle; 
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        
        double degReference = angle.getDegrees();
     
       
        
        controller.setReference (degReference, ControlType.kPosition);
        
    }

   

    private Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    public Rotation2d getCanCoder()
    {
        
        return Rotation2d.fromDegrees((angleEncoder.getAbsolutePosition().getValue().in(Degrees))); //*360 
        //return getAngle();
    }

    public int getModuleNumber() 
    {
        return moduleNumber;
    }

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
