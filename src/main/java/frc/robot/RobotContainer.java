package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.util.LimelightHelpers;
import frc.robot.commands.defaultcommands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Autonomous Chooser */
    private SendableChooser<Command> chooser;

    /* Event Loop */
    // private final EventLoop eventLoop = new EventLoop();

    /* Drive Controls */
    private int translationAxis = XboxController.Axis.kLeftY.value;
    private int strafeAxis = XboxController.Axis.kLeftX.value;
    private int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Controls */
    // private int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    // private int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;

    private double savedLimelightX;

    private PhotonCamera camera = new PhotonCamera("camera1");
    private double rotation, savedCameraX;


    

    /* Field Oriented Toggle */
    private boolean isFieldOriented = false;

    /* Driver Buttons */
    //private final JoystickButton flipAxes = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
    
    /* Operator Buttons */
    private final JoystickButton elevatorUpButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton elevatorDownButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton climberDownButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton climberUpButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton aIntakeUpButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton aIntakeDownButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton aIntakeInButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton aIntakeOutButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rGL4Button = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton rGLOtherButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rGOpenButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final AlgaeIntake aIntake = new AlgaeIntake();
    private final RainGutter rainGutter = new RainGutter();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        
        
        if(translationAxis < Math.abs(0.1)){
            translationAxis = 0;
        }
        if(strafeAxis < Math.abs(0.1)){
            strafeAxis = 0;
        }
        if(rotationAxis < Math.abs(0.1)){
            rotationAxis = 0;
        }    

        swerve.setDefaultCommand(
            new DefaultSwerve(
                swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
                )
        );

        climber.setDefaultCommand(new DefaultClimber(climberUpButton, climberDownButton, climber));
        elevator.setDefaultCommand(new DefaultElevator(elevatorUpButton, elevatorDownButton, elevator));
        aIntake.setDefaultCommand(new DefaultAlgaeIntake(aIntakeUpButton, aIntakeDownButton, aIntakeInButton, aIntakeOutButton, aIntake));
        rainGutter.setDefaultCommand(new DefaultRainGutter(rGL4Button, rGLOtherButton, rGOpenButton, rainGutter));

        // flipAxes.whileTrue(
        //     new DefaultSwerve(
        //         swerve, 
        //         () -> -driver.getRawAxis(translationAxis), 
        //         () -> 0.0,//-driver.getRawAxis(strafeAxis), 
        //         () -> getCameraRotation(), 
        //         () -> true
        //         )
        // );
     
    
        //Initalize Autonomous Chooser
        chooser = new SendableChooser<Command>();

        // Configure the button bindings
        configureButtonBindings();

        //Initialize Autonomous Chooser
        initializeAutoChooser();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.resetEverything()));
        robotCentric.toggleOnTrue(new InstantCommand(() -> toggleRobotCentric()));
    }

    public void toggleRobotCentric(){
        isFieldOriented = !isFieldOriented;
    }

    // FOV is 30
    // Max rotation we want is .5
    public double getCameraRotation(){
        var result = camera.getLatestResult();

        if(!result.hasTargets()){
            rotation = 0;
          }
          else{
            PhotonTrackedTarget target = result.getBestTarget();
            rotation = target.getYaw();
          }

          if(rotation != 0){
            savedCameraX = rotation;
          }
        else if ((rotation != 0 && Math.abs(rotation) < 0.1)){
            return 0;
        }
        return -(savedCameraX/60);
          
    }

    public void initializeAutoChooser() {
        //double initRoll = swerve.getRoll();

        chooser.setDefaultOption("Nothing", null);
        // chooser.addOption("Test Auto", new testAuto(swerve));
        SmartDashboard.putData(chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}