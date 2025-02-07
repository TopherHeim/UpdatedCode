package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;

public class Climber extends SubsystemBase implements ArmConstants{
    public SparkMax climbMotor;

    public Climber(){
        climbMotor = new   SparkMax(climbId, MotorType.kBrushless);
    }

    public void climbUp(double s){
        climbMotor.set(s);
    }

    public void climbDown(double s){
        climbMotor.set(s);
    }
}
