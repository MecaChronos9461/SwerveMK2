// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/RunCommand.h>

#include<frc/Joystick.h>
#include "Constants.h"

#include "subsystems/SwerveSubsystem.h"

#include "subsystems/IntakeSubsystem.h"

#include "subsystems/ArmSubsystem.h"

#include "subsystems/ShooterSubsystem.h"

#include "subsystems/ClimberSubsystem.h"

#include "frc2/command/Command.h"

#include "commands/IntakeCommand/ArmCommand.h"

#include "frc2/command/button/JoystickButton.h"

#include <frc/XboxController.h>

#include "subsystems/ControlSubsystem.h"

//Auto Commands Library;
#include "commands/SwerveAutonomousCommand.h"

//Command Library
#include "commands/SwerveCommand.h"

#include "commands/ShooterCommand.h"

#include "commands/ClimberCommand.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();


    void AutonomousInit();
    void TeleopInit();


 private:

  // Replace with CommandPS4Controller or CommandJoystick if needed
  double Catch_Speed;
  double Spit_Speed;
 
 //WAIT COMMANDS
 
  

  // Xbox Controller
  frc2::CommandXboxController m_TopController{OperatorConstants::kTopControllerPort};
  //frc::Joystick mTopController{OperatorConstants::kTopControllerPort};

  //frc2::JoystickButton* button2 = new frc2::JoystickButton(&m_DriverJoystick,2);

  // The robot's subsystems are defined here...

  //XboxSubsystem Controller
  ControlSubsystem Control;  

  //SwerveSubsystem Object  
  SwerveSubsystem swerve_Subsystem;

  //intakeSubsystem Object  
  IntakeSubsystem intake_Subsystem;

  //Auto Command Library

  //ShooterSubsystem Object
  ShooterSubsystem shooter_Subsystem;

  //armSubsystem Object
  ArmSubsystem arm_Subsystem;

  ArmCommand armCommand{&arm_Subsystem};

  //armSubsystem Object
  ClimberSubsystem climber_Subsystem;

  ClimberCommand climberCommand{&climber_Subsystem};

  SwerveCommand Command{&swerve_Subsystem, &Control, Control.GetFieldOriented()};

  ShooterCommand shooterCommand{&shooter_Subsystem};

  void ConfigureBindings();
};
