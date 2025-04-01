package frc.robot.subsystems;

import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    private final SwerveDriveKinematics kinematics;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private boolean lastPoseLimelight = false;
    private Pose2d lastPose = new Pose2d();
    
    /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator;

    public Swerve() {

        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k200Hz); // TODO: check if im allowed to do this
        gyro.reset();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        kinematics = new SwerveDriveKinematics(
                Constants.Swerve.flModuleOffset,
                Constants.Swerve.frModuleOffset,
                Constants.Swerve.blModuleOffset,
                Constants.Swerve.brModuleOffset);

        m_poseEstimator =       
        new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              mSwerveMods[0].getPosition(),
              mSwerveMods[1].getPosition(),
              mSwerveMods[2].getPosition(),
              mSwerveMods[3].getPosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
  

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        /* PathPlanner stuff */

        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPosePP, // Robot pose supplier
                    this::resetOdometryPP, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelativePP(speeds), // Method that will drive the robot given
                                                                            // ROBOT RELATIVE ChassisSpeeds. Also
                                                                            // optionally outputs individual module
                                                                            // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        }

        catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    /* PathPlanner Commands */
    public void driveRobotRelativePP(ChassisSpeeds robotRelativeSpeeds) {
        // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds,
        // 0.02);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(robotRelativeSpeeds);// targetSpeeds);
        setStatesPP(targetStates);
    }

    public void setStatesPP(SwerveModuleState[] targetStates) {
        // SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
        // Constants.Swerve.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setTargetStatePP(targetStates[i]);
        }
        // System.out.println("setStatesPP: " +
        // " vX: " + kinematics.toChassisSpeeds(targetStates).vxMetersPerSecond +
        // " vY: " + kinematics.toChassisSpeeds(targetStates).vyMetersPerSecond +
        // " Rad/s: " + kinematics.toChassisSpeeds(targetStates).omegaRadiansPerSecond);
    }

    public void resetOdometryPP(Pose2d pose) {
        m_poseEstimator.resetPose(pose);
    }

    /* Other methods */

    public void driveSlow(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            double maxSpeed) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveStraight(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        Rotation2d.fromDegrees(0))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    public Pose2d getPose() {
        // SmartDashboard.putNumber("XPose", swerveOdometry.getPoseMeters().getX());
        // SmartDashboard.putNumber("Ypose", swerveOdometry.getPoseMeters().getY());
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getPosePP(){
        m_poseEstimator.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(),
                mSwerveMods[1].getPosition(),
                mSwerveMods[2].getPosition(),
                mSwerveMods[3].getPosition()
            });

        boolean doRejectUpdate = false;

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
            doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
            doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            //m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5, 999999));
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 1));
            m_poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }

        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry() {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), new Pose2d());
    }

    public void setOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // System.out.println("getSpeeds: " +
        // " vX: " + kinematics.toChassisSpeeds(getModuleStates()).vxMetersPerSecond +
        // " vY: " + kinematics.toChassisSpeeds(getModuleStates()).vyMetersPerSecond +
        // " rad/s: " +
        // kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " position", mod.getPosition().distanceMeters);
        }
        return positions;
    }

    public void resetEverything() {
        zeroGyro();
        resetOdometry();
        // resetModulesToAbsolute();
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public double getDistanceSensor() {
        return 0;
    }

    public void getEmRight() {
        for (SwerveModule mod : mSwerveMods) {
            if ((Math.abs(mod.getAbsoluteAngle().getDegrees()) > 1)
                    && (Math.abs(mod.getAbsoluteAngle().getDegrees()) < 359)) {
                mod.getItRight();
            }
        }
        // drive(new Translation2d(0, 0), 0, true, true);
    }

    public boolean checkRight() {
        boolean right = true;
        for(SwerveModule mod : mSwerveMods){
            if((Math.abs(mod.getAbsoluteAngle().getDegrees()) > 1) && (Math.abs(mod.getAbsoluteAngle().getDegrees()) < 359)){
                right = false;
            }
        }
        return right;
    }

    // public void updateOffset(){
    // for(SwerveModule mod: mSwerveMods){
    // mod.updateAngleOffset();
    // SmartDashboard.putNumber("angle offset mod" + mod.moduleNumber,
    // mod.getAngleOffset());
    // }
    // }

    public double getSpeeds(){
        double averageAbsSpeed = 0;
        for(SwerveModule mod: mSwerveMods){
            averageAbsSpeed += mod.getDriveEncoder().getVelocity();
        }
        return (averageAbsSpeed / 4.0);
    }

    @Override
    public void periodic() {

        swerveOdometry.update(getYaw(), getModulePositions()); 
        SmartDashboard.putNumber("yaw: ", getYaw().getDegrees());
        SmartDashboard.putNumber("XPose", getPosePP().getX());
        SmartDashboard.putNumber("Ypose", getPosePP().getY());

        SmartDashboard.putNumber("speeds", getSpeeds());
        
        var alliance = DriverStation.getAlliance();
        SmartDashboard.putBoolean("Alliance is red?", alliance.get() == DriverStation.Alliance.Red);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());            
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getState().angle.getDegrees());       

        }

    }
}
