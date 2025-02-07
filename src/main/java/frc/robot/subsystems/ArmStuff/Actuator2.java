package frc.robot.subsystems.ArmStuff;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj.PWM;

public class Actuator2 extends SubsystemBase implements ArmConstants {
    public PWM actuator;

    public Actuator2(){
        actuator = new PWM(0);
    }

    public void moveActuator2(double s){
        actuator.setPosition(s);
    }

    public double actuatorPos(){
        return actuator.getPosition();
    }

}
 