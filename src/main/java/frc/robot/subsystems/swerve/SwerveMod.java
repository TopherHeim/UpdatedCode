
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
        
        /////////////// mod 0 ////////////////////
        if (moduleNumber == 0){
        double targetMin0 = 148;
        double targetMax0 = 154;
        double targetException0 = 151;

        while (getCanCoder().getDegrees() < targetMin0 || (getCanCoder().getDegrees() > targetMax0 && getCanCoder().getDegrees() != targetException0)) {
            double currentAngle0 = getCanCoder().getDegrees();
            double distanceToTargetMin0 = Math.abs(currentAngle0 - targetMin0);
            double distanceToTargetMax0 = Math.abs(currentAngle0 - targetMax0);
    
            double distanceToTarget0 = Math.min(distanceToTargetMin0, distanceToTargetMax0);
    
            double motorSpeed = 0.1; // Default speed
    
            if (distanceToTarget0 < 5) {
                motorSpeed = 0.025; 
            } else if (distanceToTarget0 < 10) {
                motorSpeed = 0.035; 
            }
            mAngleMotor.set(motorSpeed);
        }
        while (getCanCoder().getDegrees() < 150 || getCanCoder().getDegrees() > 152) {
            double currentAngle0 = getCanCoder().getDegrees();
    
            double distanceToTarget0 = Math.abs(currentAngle0 - 151); 
    
            double motorSpeed = 0.025;
            if (distanceToTarget0 < 1) {
                motorSpeed = 0.01;
            } else if (distanceToTarget0 < 2) {
                motorSpeed = 0.02; 
            }
            mAngleMotor.set(motorSpeed);
        }
        if (getCanCoder().getDegrees() >= 150 && getCanCoder().getDegrees() <= 152) {
            mAngleMotor.set(0); // Stop motor
            encoder.setPosition(0); // Reset encoder position
        }
        }
        //////////////////////////////////

        /////////////// mod 1 ////////////////////
        if (moduleNumber == 1){
        double targetMin1 = 38;
        double targetMax1 = 44;
        double targetException1 = 41;

        while (getCanCoder().getDegrees() < targetMin1 || (getCanCoder().getDegrees() > targetMax1 && getCanCoder().getDegrees() != targetException1)) {
            double currentAngle1 = getCanCoder().getDegrees();
            double distanceToTargetMin1 = Math.abs(currentAngle1 - targetMin1);
            double distanceToTargetMax1 = Math.abs(currentAngle1 - targetMax1);
    
            double distanceToTarget1 = Math.min(distanceToTargetMin1, distanceToTargetMax1);
    
            double motorSpeed = 0.1; // Default speed
    
            if (distanceToTarget1 < 5) {
                motorSpeed = 0.025; 
            } else if (distanceToTarget1 < 10) {
                motorSpeed = 0.035; 
            }
            mAngleMotor.set(motorSpeed);
        }
        while (getCanCoder().getDegrees() < 40 || getCanCoder().getDegrees() > 42) {
            double currentAngle1 = getCanCoder().getDegrees();
    
            double distanceToTarget1 = Math.abs(currentAngle1 - 41); 
    
            double motorSpeed = 0.025;
            if (distanceToTarget1 < 1) {
                motorSpeed = 0.01;
            } else if (distanceToTarget1 < 2) {
                motorSpeed = 0.02; 
            }
            mAngleMotor.set(motorSpeed);
        }
        if (getCanCoder().getDegrees() >= 40 && getCanCoder().getDegrees() <= 42) {
            mAngleMotor.set(0); // Stop motor
            encoder.setPosition(-180); // Reset encoder position
        }
        }
        //////////////////////////////////

        /////////////// mod 2 ////////////////////
        if (moduleNumber == 2){
        double targetMin2 = 93;
        double targetMax2 = 99;
        double targetException2 = 96;

        while (getCanCoder().getDegrees() < targetMin2 || (getCanCoder().getDegrees() > targetMax2 && getCanCoder().getDegrees() != targetException2)) {
            double currentAngle2 = getCanCoder().getDegrees();
            double distanceToTargetMin2 = Math.abs(currentAngle2 - targetMin2);
            double distanceToTargetMax2 = Math.abs(currentAngle2 - targetMax2);
    
            double distanceToTarget2 = Math.min(distanceToTargetMin2, distanceToTargetMax2);
    
            double motorSpeed = 0.1; // Default speed
    
            if (distanceToTarget2 < 5) {
                motorSpeed = 0.025; 
            } else if (distanceToTarget2 < 10) {
                motorSpeed = 0.035; 
            }
            mAngleMotor.set(motorSpeed);
        }
        while (getCanCoder().getDegrees() < 95 || getCanCoder().getDegrees() > 97) {
            double currentAngle2 = getCanCoder().getDegrees();
    
            double distanceToTarget2 = Math.abs(currentAngle2 - 96); 
    
            double motorSpeed = 0.025;
            if (distanceToTarget2 < 1) {
                motorSpeed = 0.01;
            } else if (distanceToTarget2 < 2) {
                motorSpeed = 0.02; 
            }
            mAngleMotor.set(motorSpeed);
        }
        if (getCanCoder().getDegrees() >= 95 && getCanCoder().getDegrees() <= 97) {
            mAngleMotor.set(0); // Stop motor
            encoder.setPosition(0); // Reset encoder position
        }
        }
        //////////////////////////////////

        /////////////// mod 3 ////////////////////
        if (moduleNumber == 3){
        double targetMin3 = 118;
        double targetMax3 = 124;
        double targetException3 = 121;

        while (getCanCoder().getDegrees() < targetMin3 || (getCanCoder().getDegrees() > targetMax3 && getCanCoder().getDegrees() != targetException3)) {
            double currentAngle3 = getCanCoder().getDegrees();
            double distanceToTargetMin3 = Math.abs(currentAngle3 - targetMin3);
            double distanceToTargetMax3 = Math.abs(currentAngle3 - targetMax3);
    
            double distanceToTarget3 = Math.min(distanceToTargetMin3, distanceToTargetMax3);
    
            double motorSpeed = 0.1; // Default speed
    
            if (distanceToTarget3 < 5) {
                motorSpeed = 0.025; 
            } else if (distanceToTarget3 < 10) {
                motorSpeed = 0.035; 
            }
            mAngleMotor.set(motorSpeed);
        }
        while (getCanCoder().getDegrees() < 120 || getCanCoder().getDegrees() > 122) {
            double currentAngle3 = getCanCoder().getDegrees();
    
            double distanceToTarget3 = Math.abs(currentAngle3 - 121); 
    
            double motorSpeed = 0.025;
            if (distanceToTarget3 < 1) {
                motorSpeed = 0.01;
            } else if (distanceToTarget3 < 2) {
                motorSpeed = 0.02; 
            }
            mAngleMotor.set(motorSpeed);
        }
        if (getCanCoder().getDegrees() >= 120 && getCanCoder().getDegrees() <= 122) {
            mAngleMotor.set(0); // Stop motor
            encoder.setPosition(180); // Reset encoder position
        }
        }
        //////////////////////////////////

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
