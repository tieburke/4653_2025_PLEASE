// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Elevator;

public class AIntakeUpAuto extends Command {
  /** Creates a new TimedDriveOut. */
  private AlgaeIntake algaeIntake;
  private Timer timer;

  public AIntakeUpAuto(AlgaeIntake aIntake) {
    algaeIntake = aIntake;
    this.timer = new Timer();

    addRequirements(algaeIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntake.setPosition(Constants.AlgaeIntake.verticalPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Constants.Elevator.positionL0 < -3.3) || timer.get() > 2);
  }
}