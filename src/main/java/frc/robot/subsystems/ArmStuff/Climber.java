package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;

public class Climber extends SubsystemBase implements ArmConstants{
    public SparkMax climbMotor1;
    public SparkMax climbMotor2;

    public Climber(){
        climbMotor1 = new SparkMax(climbId1, MotorType.kBrushless);
        climbMotor2 = new SparkMax(climbId2, MotorType.kBrushless);
    }

    public void climbUp(double s){
        climbMotor1.set(s);
        climbMotor2.set(s);
    }

    public void climbD1(double s){
        climbMotor1.set(s);
    }

    public void climbD2(double s){
        climbMotor2.set(s);
    }
}
