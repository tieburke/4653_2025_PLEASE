// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class AlignToReefTagRelativeL extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer, overallTimer;
  private Swerve swerve;
  private double tagID = -1;
  private boolean into = false;
  private boolean out = true;

  public AlignToReefTagRelativeL(Swerve swerve) {
    xController = new PIDController(2.8, 0.0, 0);  // Vertical movement
    yController = new PIDController(3, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(0.058, 0, 0);  // Rotation
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    into = true;
    out = false;

    SmartDashboard.putBoolean("intoTheAlignment (out)", out);
    SmartDashboard.putBoolean("intoTheAlignment (into)", into);

    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.overallTimer = new Timer();
    this.overallTimer.start();

    rotController.setSetpoint(-5);
    rotController.setTolerance(1);

    xController.setSetpoint(-0.415);
    xController.setTolerance(0.02);

    yController.setSetpoint(-0.353);
    yController.setTolerance(0.02);

    tagID = LimelightHelpers.getFiducialID("limelight-b");
  }

  @Override
  public void execute() {
    
    if (LimelightHelpers.getTV("limelight-b") && LimelightHelpers.getFiducialID("limelight-b") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-b");
      SmartDashboard.putNumber("x", positions[2]);
      SmartDashboard.putNumber("y", positions[0]);

      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      swerve.drive(new Translation2d(xSpeed, ySpeed), rotValue, false, true);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      swerve.drive(new Translation2d(), 0, false, true);
    }

  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false, true);
    if(LimelightHelpers.getFiducialID("limelight-b") != -1){
      swerve.resetOdometryPP(LimelightHelpers.getBotPose2d_wpiBlue("limelight-b"));
    }
    into = false;
    out = true;

    SmartDashboard.putBoolean("intoTheAlignment (out)", out);
    SmartDashboard.putBoolean("intoTheAlignment (into)", into);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(1) ||
        stopTimer.hasElapsed(0.3) || 
        (LimelightHelpers.getTA("limelight-b") > 18 && Math.abs(LimelightHelpers.getTX("limelight-b")) < 5) ||
        (DriverStation.isAutonomous() && overallTimer.hasElapsed(2));
    }

}