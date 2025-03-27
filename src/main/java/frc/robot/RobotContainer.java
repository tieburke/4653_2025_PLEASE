package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.sql.rowset.JoinRowSet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.*;
import frc.robot.util.LimelightHelpers;
import frc.robot.commands.defaultcommands.*;
import frc.robot.commands.autocommands.*;

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

    /* Field Oriented Toggle */
    private boolean isFieldOriented = true;

    /* Driver Buttons */
    //private final JoystickButton flipAxes = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton limelightAlignLeft = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton limelightAlignRight = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton setToZero1 = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton setToZero2 = new JoystickButton(driver, XboxController.Button.kBack.value);

    private final JoystickButton QF = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton QR = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Operator Buttons */
    private final JoystickButton elevl4Button = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton elevl3Button = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton elevl2Button = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton elevl1Button = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton resetAIntakeButton = new JoystickButton(operator, XboxController.Button.kBack.value);
    private int elevUpDown = XboxController.Axis.kRightY.value;
    private final JoystickButton climbButtonUp = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton climbButtonDown = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton aIntakeVert = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aIntakeHor = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private int aIntakeUpDown = XboxController.Axis.kLeftX.value;
    private int aIntakeInOut = XboxController.Axis.kLeftY.value;
    private int rGL4 = XboxController.Axis.kRightTrigger.value;
    private int rGLOther = XboxController.Axis.kLeftTrigger.value;
    private final JoystickButton cameraToggle = new JoystickButton(operator, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final Climber climber = new Climber();
    private final AlgaeIntake aIntake = new AlgaeIntake();
    private final RainGutter rainGutter = new RainGutter();

    public static boolean elevUp = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        

        // swerve.updateOffset();

        NamedCommands.registerCommand("L4Auto", new L4Auto(elevator));
        NamedCommands.registerCommand("RGL4Auto", new RGL4Auto(rainGutter));
        NamedCommands.registerCommand("RGResetAuto", new RGResetAuto(rainGutter));
        NamedCommands.registerCommand("L0Auto", new L0Auto(elevator));

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
                    () -> true,
                    setToZero1,
                    setToZero2,
                    limelightAlignLeft,
                    limelightAlignRight,
                    elevUp
               )
            );

        
        climber.setDefaultCommand(new DefaultClimber(climbButtonUp, climbButtonDown, cameraToggle, climber));
        elevator.setDefaultCommand(new DefaultElevator(elevl4Button, elevl3Button, elevl2Button, elevl1Button, () -> operator.getRawAxis(elevUpDown), elevator));
        aIntake.setDefaultCommand(new DefaultAlgaeIntake(aIntakeHor, aIntakeVert, resetAIntakeButton, () -> operator.getRawAxis(aIntakeInOut), () -> operator.getRawAxis(aIntakeUpDown), aIntake));
        rainGutter.setDefaultCommand(new DefaultRainGutter(() -> operator.getRawAxis(rGL4), () -> operator.getRawAxis(rGLOther), rainGutter));

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
        // QF.whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // QR.whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // setToZero1.whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // setToZero2.whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public void toggleRobotCentric(){
        isFieldOriented = !isFieldOriented;
    }

    /* Limelight Stuff */

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    public static double limelight_aim_proportional(){    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .05;
    
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    
        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;
    
        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -0.05;
    
        return targetingAngularVelocity;
    }
    
    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public static double limelight_range_proportional(){    
        double kP = -.05;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= Math.PI;
        targetingForwardSpeed *= 0.05;
        return targetingForwardSpeed;
    }

    // FOV is 30
    // Max rotation we want is .5

    public void initializeAutoChooser() {
        //double initRoll = swerve.getRoll();

        chooser.setDefaultOption("Nothing", null);

        chooser.addOption(
            "Limelight1Piece",
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new LPartiallyUp(elevator),
                    new AIntakeUpAuto(aIntake)
                ),
                new LimelightAlignAuto(swerve),
                new L4Auto(elevator),
                new RGL4Auto(rainGutter),
                new ParallelCommandGroup(
                    new L0Auto(elevator),
                    new RGResetAuto(rainGutter)
                )
            )
        );

        chooser.addOption("StupidDriveOut", 
        new SequentialCommandGroup(
            new StupidDriveForwardAuto(swerve)
        ));

        chooser.addOption("PPTest", getPathPlannerAutoTest());
        chooser.addOption("twelvePiece", getPathPlannerAutoTwelvePiece());

        //chooser.addOption("onePiece", getPathPlannerAuto());
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

    public Command getPathPlannerAutoTest(){
        return AutoBuilder.buildAuto("Test");
    }

    public Command getPathPlannerAutoTwelvePiece(){
        return AutoBuilder.buildAuto("twelvePieceAuto");
    }

    public Command getPathPlannerPath(){
        try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
        
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

}
