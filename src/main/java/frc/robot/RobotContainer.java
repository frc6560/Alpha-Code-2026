package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import swervelib.SwerveInputStream;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SubsystemManagerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utility.Shooter.ShotCalculator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoModeChooser;
import frc.robot.autonomous.AutoCommands;
import frc.robot.autonomous.AutoNames;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

     // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
    "swerve/falcon"));
    
    private final VisionSubsystem vision;

    // Subsystems
    private final Shooter shooter = new Shooter();
    private final Feeder feeder = new Feeder();

    // Shot calculator for snap-to-target heading
    private final ShotCalculator shotCalculator = new ShotCalculator();
    private boolean wasSnapModeActive = false;

    private final AutoCommands factory;
    private final AutoModeChooser autoChooser;

    // Normal drive mode - angular velocity control
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

    // Threshold for left trigger to activate snap-to-target mode
    private static final double SNAP_MODE_TRIGGER_THRESHOLD = 0.5;


    public RobotContainer() {
      // shooter.setDefaultCommand(new ShooterCommand(shooter, drivebase));
      
      factory = new AutoCommands(drivebase, shooter, feeder);

      autoChooser = new AutoModeChooser(factory);
      SmartDashboard.putData("Auto Chooser", autoChooser.getAutoChooser());

      // Shooter test RPM entry - editable in Shuffleboard
      SmartDashboard.putNumber("Shooter Test RPM", -1500);

      List<LimelightVision> limelights = new ArrayList<LimelightVision>();
      for(String name : LimelightConstants.LIMELIGHT_NAMES) {
        Pose3d cameraPose = LimelightConstants.getLimelightPose(name);
        limelights.add(new LimelightVision(drivebase, name, cameraPose));
      }

      vision = new VisionSubsystem(limelights);
      configureBindings();
    }

    private void configureBindings() {
        // Dual-mode drive command: normal or snap-to-target based on left trigger
        Command dualModeDrive = drivebase.run(() -> {
            boolean snapModeActive = driverXbox.getLeftTriggerAxis() > SNAP_MODE_TRIGGER_THRESHOLD;
            wasSnapModeActive = snapModeActive;

            // Get translation inputs (scaled and deadbanded from SwerveInputStream)
            ChassisSpeeds baseSpeeds = driveAngularVelocity.get();
            double vx = baseSpeeds.vxMetersPerSecond;
            double vy = baseSpeeds.vyMetersPerSecond;

            double omega;
            if (snapModeActive) {
              // // Snap-to-target mode: calculate heading to face the shot target
              //   Pose2d robotPose = drivebase.getPose();
              //   ChassisSpeeds fieldVelocity = drivebase.getFieldVelocity();

              //   // Update shot calculator with current pose and velocity
              //   shotCalculator.calculate(robotPose, fieldVelocity);

              //   // Convert robot-relative turret angle to field-relative heading
              //   double turretAngle = shotCalculator.getTurretAngle();
              //   double targetHeading = MathUtil.angleModulus(
              //       robotPose.getRotation().getRadians() + turretAngle
              //   );

              //   // Get profiled omega for smooth heading control
              //   omega = drivebase.calculateSnapToTargetOmega(targetHeading);
                // Snap-to-target mode: point directly at the blue hub (for LUT testing)
                Pose2d robotPose = drivebase.getPose();

                double dx = Constants.FieldConstants.BLUE_HUB_CENTER.getX() - robotPose.getX();
                double dy = Constants.FieldConstants.BLUE_HUB_CENTER.getY() - robotPose.getY();
                double targetHeading = Math.atan2(dy, dx);
                if(Math.abs(MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians())) < Math.toRadians(1.0)) {
                    omega = 0.0;
                }
                else{
                    omega = drivebase.getRotationalOutput(targetHeading).omegaRadiansPerSecond;
                }
            } else {
                // Normal mode: use angular velocity from right stick
                omega = baseSpeeds.omegaRadiansPerSecond;
            }

            drivebase.driveFieldOriented(new ChassisSpeeds(vx, vy, omega));
        });

        drivebase.setDefaultCommand(dualModeDrive);
        driverXbox.a().onTrue(
          Commands.defer(() -> {
            return Commands.runOnce(() -> vision.hardReset("limelight"), vision);
          }, Set.of(vision))
        );
        driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
        driverXbox.x().onTrue(Commands.defer(() -> drivebase.alignToTrenchCommand(), Set.of(drivebase)));
        driverXbox.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(drivebase.sysIdDriveMotorCommand()), drivebase));
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
        driverXbox.leftBumper().onTrue(Commands.run(() -> {
            double targetRPM = SmartDashboard.getNumber("Shooter Test RPM", -1500);
            shooter.setRPM(targetRPM);
        }, shooter).withTimeout(5.0).finallyDo((interrupted) -> shooter.setRPM(0)));
        
        // B button: Start feeder at constant RPM
        driverXbox.rightBumper().onTrue(Commands.run(() -> feeder.setRPM(-1800), feeder).withTimeout(5.0).finallyDo((interrupted) -> feeder.setRPM(0)));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getAutoChooser().selectedCommand();
    }
}
