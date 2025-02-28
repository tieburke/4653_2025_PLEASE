// package frc.robot.commands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;

// import static frc.robot.util.Limelight.limelight_aim_proportional;
// import static frc.robot.util.Limelight.limelight_range_proportional;

// public class ReefAlign extends Command{
//     private Swerve swerve;

//     public ReefAlign(Swerve swerveInThatCorner){
        
//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {}

//     @Override
//     public void execute(){
//     swerve.drive(new Translation2d(limelight_aim_proportional(), 0), limelight_range_proportional(), false, true);
//     }

//     @Override
//     public void end(boolean interupted){}

//     @Override
//     public boolean isFinished(){
//         return false;
//     }
// }