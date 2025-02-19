package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Elevator;

public class ElevatorRun extends Command{
    
    private Elevator elevator;
    private DoubleSupplier elevatorRun;
    private DoubleSupplier wristMove;

    public ElevatorRun(Elevator elevator, DoubleSupplier elevatorRun, DoubleSupplier wristMove){
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevatorRun = elevatorRun;
        this.wristMove = wristMove;
    }


    @Override
    public void execute(){
        
        if (elevatorRun.getAsDouble() >= 0.075){
            elevator.moveElevator(elevatorRun.getAsDouble());
            SmartDashboard.putNumber("value", elevatorRun.getAsDouble());
        }
        else if (elevatorRun.getAsDouble() <= -0.075){
            elevator.moveElevator(elevatorRun.getAsDouble());
        }
        else {
            elevator.moveElevator(0);
        }



        if (wristMove.getAsDouble() >= 0.075){
            elevator.moveWrist(wristMove.getAsDouble());
            //SmartDashboard.putNumber("value", wristMove.getAsDouble());
        }
        else if (wristMove.getAsDouble() <= -0.075){
            elevator.moveWrist(wristMove.getAsDouble());
        }
        else {
            elevator.moveWrist(0);
        }
    }
}
