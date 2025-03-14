// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import frc.robot.commands.defaultcommands.DefaultClimber;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private UsbCamera camera;
  private UsbCamera camera2;
  VideoSink server;
  private boolean toggle = false;

  private Command m_autonomousCommand;

  // private String trajectory1JSON = "paths/MoveToFirstBlock.wpilib.json";
  // private Trajectory trajectory1 = new Trajectory();

  // private String trajectory2JSON = "MoveToFirstCone.wpilib.json";
  // private Trajectory trajectory2 = new Trajectory();

  private RobotContainer m_robotContainer;
  // private UsbCamera camera;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

// PWM port 2
// Must be a PWM header, not MXP or DIO
AddressableLED m_led = new AddressableLED(9);
//AddressableLED m_led1 = new AddressableLED(3);

// Reuse buffer
// Default to a length of 60, start empty output
// Length is expensive to set, so only set it once, then just update data
AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(310-199);
// AddressableLEDBuffer m_ledBuffer1 = new AddressableLEDBuffer(60);

  @Override
  public void robotInit() {

m_led.setLength(m_ledBuffer.getLength());
// m_led1.setLength(m_ledBuffer1.getLength());

//LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kDarkGreen, Color.kWhite);
//LEDPattern pattern = base.scrollAtAbsoluteSpeed(InchesPerSecond.of(10), Meters.of(1.0/60));
LEDPattern pattern = LEDPattern.solid(Color.kDarkGreen);
// Apply the LED pattern to the data buffer
pattern.applyTo(m_ledBuffer);
// pattern.applyTo(m_ledBuffer1);

// Set the data
m_led.setData(m_ledBuffer);
// m_led1.setData(m_ledBuffer1);
m_led.start();
// m_led1.start();
	  
    //Initialize USB camera
    for(int port = 5800; port <= 5805; port++){
      PortForwarder.add(port, "limelight.local", port);
    }

    // http://roborio-4860-frc.local:5801/

    
    // camera = new UsbCamera("USBCam", 1);

    // CameraServer.startAutomaticCapture();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    camera = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();

    // try {
    //   Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
    //   trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectory1JSON, ex.getStackTrace());
    // }

    // try {
    //   Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON);
    //   trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
    // } catch (IOException ex) {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectory2JSON, ex.getStackTrace());
    // }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  
	@Override
	public void autonomousInit() {
		if(m_robotContainer.getAutonomousCommand() != null) {
			m_robotContainer.getAutonomousCommand().schedule();

			// Reset Odometry and Gyro
			//robotContainer.getResetCommand().schedule();
		}
	}

  /** This function is called periodically during autonomous. */
  @Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      if(!DefaultClimber.cameraSwitch){
        server.setSource(camera);
        toggle = true;
      }
      else{
        System.out.println("Setting camera");
        server.setSource(camera2);
        toggle = false;
      }
      SmartDashboard.putBoolean("cameraToggle: ", DefaultClimber.cameraSwitch);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
