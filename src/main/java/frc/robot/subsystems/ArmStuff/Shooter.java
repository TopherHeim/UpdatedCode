package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase implements ArmConstants  {

    public SparkMax ringShooter1;
    public SparkMax ringShooter2;
    

    public Shooter(){
        ringShooter1 = new SparkMax(shooterId1, MotorType.kBrushless);
        ringShooter2 = new SparkMax(shooterId2, MotorType.kBrushless);
    }

    public void setSpeed(double s){
        ringShooter1.set(s);
        ringShooter2.set(s * -1);
    }
    /* 
    public void getVelo(){
        
    }
    */


    
}
