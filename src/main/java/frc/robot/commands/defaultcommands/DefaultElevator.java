package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DefaultElevator extends Command{
    private Elevator elevator;
    private BooleanSupplier L1, L2, L3, L4;
    private DoubleSupplier manualDrive;

    public DefaultElevator( BooleanSupplier l4, BooleanSupplier l3, BooleanSupplier l2, BooleanSupplier l1, DoubleSupplier manualdrive, Elevator elevatorThing){
        L1 = l1;
        L2 = l2;
        L3 = l3;
        L4 = l4;
        manualDrive = manualdrive;
        elevator = elevatorThing;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        if(L1.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL1);
        }
        
        if(L2.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL2);
        }

        if(L3.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL3);
        }

        if(L4.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL4);
        }

        if(manualDrive.getAsDouble() < -0.1){
            elevator.elevatorUpManual();
        }

        else if(manualDrive.getAsDouble() > 0.1){
            elevator.elevatorDownManual();
        }

        else if (!L4.getAsBoolean() && !L1.getAsBoolean() && !L2.getAsBoolean() && !L3.getAsBoolean() && !L4.getAsBoolean()){
            elevator.setPosition(Constants.Elevator.positionL0);
        }
    }

    @Override
    public void end(boolean interupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}