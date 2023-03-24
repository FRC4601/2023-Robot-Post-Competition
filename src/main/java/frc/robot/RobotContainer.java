//CB-10 "INSERT ROBOT NAME HERE"

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Import all (*) constants, subsystems and commands
import frc.robot.commands.*;
import frc.robot.commands.autos.MidCubeAndDriveOut;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {
  // The robot's subsystems and commands? are defined here...
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static Arm m_arm = new Arm();
  public static Vision m_vision = new Vision();
  public static Claw m_claw = new Claw();


  public static final Joystick leftstick = new Joystick(0);
  public static final Joystick rightstick = new Joystick (2);
  public static final XboxController xbox = new XboxController(1);

  //Smartdashboard choosers/data
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */ //Things that should happen when the robot first initializes
  public RobotContainer() {
    configureBindings(); 
    configureSmartdashboard(); 

    //Set Default Commands
    m_drivetrain.setDefaultCommand(new ArcadeDrive()); 
    m_arm.setDefaultCommand(new RunArm());
    m_claw.setDefaultCommand(new RunClaw());
  }

  /** Use this method to define your trigger->command mappings. Triggers can be created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@lin CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flightjoysticks}. */
  private void configureBindings() {
    //Pilot Controls
  
    //Xbox controls
    new Trigger(() -> xbox.getAButton()).onTrue(new InstantCommand(() -> m_arm.ResetArmEncoders()));
    new Trigger(() -> xbox.getBButton()).onTrue(new InstantCommand(() -> m_drivetrain.ResetGyro()));

    new Trigger(() -> xbox.getStartButton()).onTrue(new InstantCommand(() -> m_claw.CloseClaw()));
    new Trigger(() -> xbox.getBackButton()).onTrue(new InstantCommand(() -> m_claw.OpenClaw()));


  }

  private void configureSmartdashboard(){
    //Smartdashboard AutoChooser options
    m_autoChooser.setDefaultOption("No Auto Selected", null);
    m_autoChooser.addOption("test auto", new MidCubeAndDriveOut());
    SmartDashboard.putData("Auto Mode", m_autoChooser); // Add chooser for auto

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected(); //Gets the autonomous mode selected on smartdashboard
  }

  
}
