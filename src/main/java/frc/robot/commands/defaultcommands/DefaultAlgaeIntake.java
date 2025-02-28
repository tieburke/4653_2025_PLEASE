package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class DefaultAlgaeIntake extends Command{
    private AlgaeIntake aIntake;
    private BooleanSupplier horizontal, vertical, manualUp, manualDown, aIn, aOut;

    public DefaultAlgaeIntake(BooleanSupplier manualDriveUp, BooleanSupplier manualDriveDown, BooleanSupplier algaeIn, BooleanSupplier algaeOut, AlgaeIntake algaeIntake){
        // horizontal = horizontalPos;
        // vertical = verticalPos;
        manualUp = manualDriveUp;
        manualDown = manualDriveDown;
        aIn = algaeIn;
        aOut = algaeOut;
        aIntake = algaeIntake;

        addRequirements(aIntake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute(){
        // if(climb.getAsBoolean()){
        //     climber.setPosition(Constants.Climber.positionClimb);
        // }

        if(manualUp.getAsBoolean()){
            aIntake.intakeUpManual();;
        }

        else if(manualDown.getAsBoolean()){
            aIntake.intakeDownManual();
        }

        else{ //if((!horizontal.getAsBoolean()) || (!vertical.getAsBoolean())){
            aIntake.intakeStop();
        }

        if(aIn.getAsBoolean()){
            aIntake.algaeIn();
        }

        else if(aOut.getAsBoolean()){
            aIntake.algaeOut();
        }

        else{
            aIntake.inOutStop();
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}