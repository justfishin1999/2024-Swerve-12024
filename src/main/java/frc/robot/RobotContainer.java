package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver,1); //A
    private final JoystickButton robotCentric = new JoystickButton(driver,2); //B
    private final JoystickButton shootSpeaker = new JoystickButton(driver,5); //LB
    private final JoystickButton shootAmp = new JoystickButton(driver,6); //RB
    private final JoystickButton runIndex = new JoystickButton(driver,4); //Y

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Indexer s_Indexer = new Indexer();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis)*0.25, 
                () -> -driver.getRawAxis(strafeAxis)*0.25, 
                () -> -driver.getRawAxis(rotationAxis)*0.25, 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /*Create binding for shooting speaker */
        shootSpeaker.whileTrue(new ShootSpeaker(s_Shooter,Constants.ShooterConstants.combined_shooterVelo));
        shootSpeaker.whileFalse(new ShootSpeaker(s_Shooter,0));

        /*Create binding for shooting amp */
        shootAmp.whileTrue(new ShootAmp(s_Shooter,Constants.ShooterConstants.top_shooterVelo,Constants.ShooterConstants.bottom_shooterVelo));
        shootAmp.whileFalse(new ShootAmp(s_Shooter,0,0));

        /*Create binding for running indexer */
        runIndex.whileTrue(new RunIndexer(s_Indexer, Constants.IndexerConstants.indexVelo));
        runIndex.whileFalse(new RunIndexer(s_Indexer, 0));
    
    }

    /**P
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
