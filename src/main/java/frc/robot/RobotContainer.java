package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterBase;
import frc.robot.subsystems.Tourelle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants.IntakeConstants;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController m_driverController = new CommandXboxController(
            DriveConstants.kXboxControllerID);
    private final CommandJoystick m_turnStick = new CommandJoystick(DriveConstants.kTurnStickID);
    private final CommandJoystick m_throttleStick = new CommandJoystick(DriveConstants.kThrottleStickID);

    private double driverMulti = 1;

    private final SendableChooser<Command> m_chooser;

    /* Subsystems */
    private Base m_base;
    private Climb m_climb;
    private ShooterBase m_shooterBase;
    private Shooter m_shooter;
    private Tourelle m_tourelle;
    private Intake m_intake;
    private Conveyor m_conveyor;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_base = new Base();
        m_climb = new Climb();
        m_shooterBase = new ShooterBase();
        m_shooter = new Shooter();
        m_tourelle = new Tourelle();
        m_intake = new Intake();
        m_conveyor = new Conveyor();

        if (m_base != null) {
            m_base.setDefaultCommand(getBaseDefaultCommand());
        }

        registerNamedCommands();

        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("AutoChooser", m_chooser);

        configureButtonBindings();
    }

    private void registerNamedCommands() {
        // pathplanner named commands go here
    }

    private Command getBaseDefaultCommand() {
        return new RunCommand(() -> {
            // double dir_x = m_driverController.getLeftX();
            // double dir_y = m_driverController.getLeftY();

            double dir_x = m_throttleStick.getX();
            double dir_y = m_throttleStick.getY();

            // Convert cartesian vector to polar for circular deadband
            double dir_r = Math.sqrt(Math.pow(dir_x, 2) + Math.pow(dir_y, 2)); // norm of vector
            double dir_theta = Math.atan2(dir_y, dir_x); // direction of vector (rad)

            // Cap norm and add deadband
            if (dir_r < DriveConstants.kControllerMovementDeadband) {
                dir_r = 0.0;
            } else if (dir_r > 1.0) {
                dir_r = 1.0;
            } else {
                dir_r = (dir_r - DriveConstants.kControllerMovementDeadband) /
                        (1 - DriveConstants.kControllerMovementDeadband);
            }
            dir_r *= dir_r;

            double turn = 0;

            turn = MathUtil.applyDeadband(m_turnStick.getX(),
                    DriveConstants.kControllerRotationDeadband);
            turn *= (turn < 0) ? -turn : turn;

            if (m_throttleStick.button(1).getAsBoolean()) {
                driverMulti = DriveConstants.kDriverMultiSlow;
            } else {
                driverMulti = 1;
            }

            double x = dir_r * Math.sin(dir_theta);
            x *= DriveConstants.kMaxTeleopSpeed * DriveConstants.kGeneralSpeedMulti;
            if (Math.abs(x) >= DriveConstants.kMaxTeleopSpeed * driverMulti) {
                x = DriveConstants.kMaxTeleopSpeed * driverMulti * (x < 0 ? -1 : 1);
            }

            double y = dir_r * Math.cos(dir_theta);
            y *= DriveConstants.kMaxTeleopSpeed * DriveConstants.kGeneralSpeedMulti;
            if (Math.abs(y) >= DriveConstants.kMaxTeleopSpeed * driverMulti) {
                y = DriveConstants.kMaxTeleopSpeed * driverMulti * (y < 0 ? -1 : 1);
            }

            turn *= DriveConstants.kMaxTeleopRotateSpeed * DriveConstants.kGeneralSpeedMulti;
            if (Math.abs(turn) >= DriveConstants.kMaxTeleopRotateSpeed * driverMulti) {
                turn = DriveConstants.kMaxTeleopRotateSpeed * driverMulti * (turn < 0 ? -1 : 1);
            }

            m_base.drive(new Translation2d(x, y), turn, false);
        }, m_base);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        m_turnStick.button(1).whileTrue(
                new StartEndCommand(() -> m_base.switchToRobotRelative(), () -> m_base.switchToFieldRelative()));

        m_turnStick.button(2).onTrue(new InstantCommand(() -> m_base.resetGyroOffset(false)));

        m_turnStick.button(5).whileTrue(new RunCommand(() -> {
            var latestMeasurement = m_base.getAveragePoseFromCameras();
            if (latestMeasurement.isPresent()) {
                m_base.setPose(latestMeasurement.get());
            }
        }));

        m_turnStick.button(6).onTrue(new InstantCommand(() -> m_base.resetGyroOffset(true)));
        /* Copilot Buttons */

        // m_driverController.povUp().and(m_driverController.leftTrigger()).whileTrue(
        //  new ManualClimb(m_climb, ClimbConstants.kManualSpeed));
        //  m_driverController.povDown().and(m_driverController.leftTrigger()).whileTrue(
        //  new ManualClimb(m_climb, -ClimbConstants.kManualSpeed));

        //  m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_shooterBase.ShooterBaseWheelsIn()));
        //  m_driverController.rightTrigger().onFalse(new InstantCommand(() -> m_shooterBase.ShooterBaseWheelOff()));
        //  m_driverController.leftTrigger().onTrue(new InstantCommand(() -> m_shooterBase.ShooterBaseWheelsOut()));
        //  m_driverController.leftTrigger().onFalse(new InstantCommand(() -> m_shooterBase.ShooterBaseWheelOff()));

         m_driverController.x().onTrue(new InstantCommand(() -> m_shooter.setMotorSpeed(4000)));
         m_driverController.y().onTrue(new InstantCommand(() -> m_shooter.stopMotors()));

        //  m_driverController.b().onTrue(new InstantCommand(() -> m_tourelle.setMotorSpeed(0.1)));
        //  m_driverController.b().onFalse(new InstantCommand(() -> m_tourelle.safeStop()));
        //  m_driverController.a().onTrue(new InstantCommand(() -> m_tourelle.setMotorSpeed(-0.1)));
        //  m_driverController.a().onFalse(new InstantCommand(() -> m_tourelle.safeStop()));

        
         m_driverController.a().onTrue(new InstantCommand(() -> m_intake.setMotorSpeed(IntakeConstants.motorSpeed)));
         m_driverController.a().onFalse(new InstantCommand(() -> m_intake.safeStop()));
         m_driverController.b().onTrue(new InstantCommand(() -> m_intake.setMotorSpeed(-IntakeConstants.motorSpeed)));
         m_driverController.b().onFalse(new InstantCommand(() -> m_intake.safeStop()));

        //  m_driverController.b().onTrue(new InstantCommand(() -> m_conveyor.converyorWheelsIn()));
        //  m_driverController.b().onFalse(new InstantCommand(() -> m_conveyor.ConveyorWheelOff()));
        //  m_driverController.a().onTrue(new InstantCommand(() -> m_conveyor.conveyorWheelsOut()));
        //  m_driverController.a().onFalse(new InstantCommand(() -> m_conveyor.ConveyorWheelOff()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // chosen autonomous command will be run
        return m_chooser.getSelected();
    }

    public void resetGyroOffsetEstimatedPose() {
        m_base.resetGyroOffset(true);
    }

    public void setNeutralModeSwerve(NeutralModeValue neutralMode) {
        m_base.setNeutralMode(neutralMode);
    }

    public void setLimelight4IMUMode(VisionConstants.LimelightIMUModes mode) {
        m_base.setLimelight4IMUMode(mode);
    }
}