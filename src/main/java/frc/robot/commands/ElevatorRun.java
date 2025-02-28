package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Elevator;

public class ElevatorRun extends Command{
    
    private Elevator elevator;
    private DoubleSupplier elevatorRun;
    private DoubleSupplier wristMove;
    private BooleanSupplier wSetPoint1;
    private BooleanSupplier wSetPoint2;
    private BooleanSupplier eSetPoint1;

    public ElevatorRun(Elevator elevator, DoubleSupplier elevatorRun, DoubleSupplier wristMove, BooleanSupplier wSetPoint1, BooleanSupplier eSetPoint1, BooleanSupplier wSetPoint2){
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevatorRun = elevatorRun;
        this.wristMove = wristMove;
        this.wSetPoint1 = wSetPoint1;
        this.wSetPoint2 = wSetPoint2;
        this.eSetPoint1 = eSetPoint1;
    }


    @Override
    public void execute(){
        
        SmartDashboard.putNumber("elevator enc", elevator.elevatorPos());
        SmartDashboard.putNumber("wrist enc", elevator.wristPos());


        if (elevatorRun.getAsDouble() >= 0.075){
            elevator.moveElevator(elevatorRun.getAsDouble());
            SmartDashboard.putNumber("value", elevatorRun.getAsDouble());
        }
        else if (elevatorRun.getAsDouble() <= -0.075){
            elevator.moveElevator(elevatorRun.getAsDouble());
        }
        else {
            elevator.moveElevator(0.01);
        }





        if (wristMove.getAsDouble() >= 0.075){
            elevator.moveWrist(wristMove.getAsDouble());
            //SmartDashboard.putNumber("value", wristMove.getAsDouble());
        }
        else if (wristMove.getAsDouble() <= -0.075){
            elevator.moveWrist(wristMove.getAsDouble());
        }
        else {
            elevator.moveWrist(0.06);
        }



        if(wSetPoint1.getAsBoolean() == true){
            elevator.wristSetPoints(-16);
            System.out.print("test");
        }

        if(wSetPoint2.getAsBoolean() == true){
            elevator.wristSetPoints(-14.6);
        }

        if(eSetPoint1.getAsBoolean() == true){
            elevator.elevatorSetPoints(180);
        }
    }
}
