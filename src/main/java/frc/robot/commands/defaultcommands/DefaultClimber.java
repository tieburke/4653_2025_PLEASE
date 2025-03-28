package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DefaultClimber extends Command {
    private Climber climber;
    private BooleanSupplier manualUp, manualDown, cameraToggle;
    public static boolean cameraSwitch;

    public DefaultClimber(BooleanSupplier manualDriveUp, BooleanSupplier manualDriveDown, BooleanSupplier cameraTog,
            Climber climberThing) {// BooleanSupplier climb
        manualUp = manualDriveUp;
        manualDown = manualDriveDown;
        cameraToggle = cameraTog;
        climber = climberThing;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (manualUp.getAsBoolean()) {
            climber.setPosition(Constants.Climber.positionClimb);
        }

        else if (manualDown.getAsBoolean()) {
            climber.climberDownManual();
            // climber.setPosition(0);
        }

        else {
            climber.climberStop();
        }

        if (cameraToggle.getAsBoolean()) {
            cameraSwitch = false;
        } else {
            cameraSwitch = true;
        }

    }

    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}