package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RainGutter;
public class RGR4Auto extends Command {
    
  /** Creates a new TimedDriveOut. */
  private RainGutter rainGutter;
  private Timer timer;

  public RGR4Auto(RainGutter rainGutter) {
    this.rainGutter = rainGutter;
    this.timer = new Timer();

    addRequirements(rainGutter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rainGutter.open();
  }

  // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//     timer.reset();
//   }

  
//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return (timer.get() > 1);
//   }
}
