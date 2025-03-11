package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ejml.equation.ManagerFunctions;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class DefaultAlgaeIntake extends Command{
    private AlgaeIntake aIntake;
    private BooleanSupplier horizontal, vertical, resetEncoder;
    private DoubleSupplier manualUpDown, aInOut;

    public DefaultAlgaeIntake(BooleanSupplier horizontalPos, BooleanSupplier verticalPos, BooleanSupplier reset, DoubleSupplier aINOUT, DoubleSupplier manualDrive, AlgaeIntake algaeIntake){
        horizontal = horizontalPos;
        vertical = verticalPos;
        resetEncoder = reset;
        manualUpDown = manualDrive;
        aInOut = aINOUT;
        aIntake = algaeIntake;

        addRequirements(aIntake);
    }

    @Override
    public void initialize() {
        aIntake.resetEncoder();
    }

    @Override
    public void execute(){
        if(horizontal.getAsBoolean()){
            // Constants.AlgaeIntake.articulateKP = 0.04;
            // aIntake.configAlgae();
            aIntake.setPosition(Constants.AlgaeIntake.horizontalPos);
        }

        if(vertical.getAsBoolean()){
            // Constants.AlgaeIntake.articulateKP = 0.08;
            // aIntake.configAlgae();
            aIntake.setPosition(Constants.AlgaeIntake.verticalPos);
        }

        if(resetEncoder.getAsBoolean()){
            aIntake.resetEncoder();
        }

        if(manualUpDown.getAsDouble() > 0.1){
            aIntake.intakeUpManual();
        }

        else if(manualUpDown.getAsDouble() < -0.1){
            aIntake.intakeDownManual();
        }


        if(aInOut.getAsDouble() > 0.1){
            aIntake.algaeOut();
        }

        else if(aInOut.getAsDouble() < -0.1){
            aIntake.algaeIn();
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