package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;

public class Actuator extends SubsystemBase implements ArmConstants {

    public SparkMax actuator;
    public RelativeEncoder enc;

    public Actuator(){
        actuator = new SparkMax(actuatorId, MotorType.kBrushless);
        enc = actuator.getEncoder();
    }

    public double getEnc(){
        return enc.getPosition();
    }

    public void moveActuator(double s){
        actuator.set(s);
    }


    
}
