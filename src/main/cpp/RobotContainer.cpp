// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/SwerveCommand.h"

#include "commands/IntakeCommand/IntakeCatchCommand.h"
#include "commands/IntakeCommand/SpitIntakeCommand.h"
#include "subsystems/ArmSubsystem.h"
#include "commands/ShooterCommand.h"
#include "subsystems/ShooterSubsystem.h"
#include "Constants.h"

#include "functional"
#include "algorithm"

#include <frc2/command/InstantCommand.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc2/command/SwerveControllerCommand.h>




RobotContainer::RobotContainer() {

swerve_Subsystem.SetDefaultCommand(Command);

arm_Subsystem.SetDefaultCommand(armCommand);

shooter_Subsystem.SetDefaultCommand(shooterCommand);

  ConfigureBindings();
  
}


void RobotContainer::ConfigureBindings() {
  
  //The default buttons bindings for arm and intake Subsystem

//-------------------------------------------------------------------------------//


  //Comandos do INTAKE
  m_TopController.A().WhileTrue(intake_Subsystem.SpitIntakeCommand(-0.35));
  m_TopController.A().OnFalse(intake_Subsystem.StopIntakeCommand());
  m_TopController.X().WhileTrue(intake_Subsystem.CatchIntakeCommand(0.35));
  m_TopController.X().OnFalse(intake_Subsystem.StopIntakeCommand());

  //Comandos do SHOOTER
  m_TopController.B().WhileTrue(shooter_Subsystem.ShootSpeakerCommand(0.1));
  m_TopController.B().OnFalse(shooter_Subsystem.StopShooterCommand());
  //m_TopController.Y().WhileTrue(shooter_Subsystem.ShootAmpCommand(ShooterConstants::speaker_ShootSpeedConstant));
  //m_TopController.Y().OnFalse(shooter_Subsystem.StopShooterCommand());

/*
  //Comandos do Braço do INTAKE
  m_TopController.LeftBumper().WhileTrue(arm_Subsystem.upArmCommand(0.2));
  m_TopController.LeftBumper().OnFalse(arm_Subsystem.StopArmCommand());
  m_TopController.RightBumper().WhileTrue(arm_Subsystem.downArmCommand(0.2));
  m_TopController.RightBumper().OnFalse(arm_Subsystem.StopArmCommand());
*/

  m_TopController.LeftTrigger().WhileTrue(climber_Subsystem.DownLeftClimberCommand(-0.2));
  m_TopController.LeftTrigger().OnFalse(climber_Subsystem.StopLeftClimberCommand());
  m_TopController.RightTrigger().WhileTrue(climber_Subsystem.DownRightClimberCommand(0.2));
  m_TopController.RightTrigger().OnFalse(climber_Subsystem.StopRightClimberCommand());

  m_TopController.LeftBumper().WhileTrue(climber_Subsystem.UpLeftClimberCommand(0.2));
  m_TopController.LeftBumper().OnFalse(climber_Subsystem.StopLeftClimberCommand());
  m_TopController.RightBumper().WhileTrue(climber_Subsystem.UpRightClimberCommand(-0.2));
  m_TopController.RightBumper().OnFalse(climber_Subsystem.StopRightClimberCommand());
  

/*
  //Comandos do CLIMBER
  m_TopController.LeftTrigger().WhileTrue(climber_Subsystem.UpClimberMotorGroupCommand(0.2));
  m_TopController.LeftTrigger().OnFalse(climber_Subsystem.StopClimberMotorGroupCommand());
  m_TopController.RightTrigger().WhileTrue(climber_Subsystem.DownClimberMotorGroupCommand(-0.2));
  m_TopController.RightTrigger().OnFalse(climber_Subsystem.StopClimberMotorGroupCommand());
  */

/*
 m_TopController.LeftBumper().ToggleOnTrue(intake_Subsystem.SpitIntakeCommand(0.2)
                        //.WithTimeout(5.0_s)
                        .AndThen(shooter_Subsystem.ShootSpeakerCommand(0.2)));


  m_TopController.LeftBumper().ToggleOnFalse(shooter_Subsystem.StopShooterCommand().
                          AlongWith(intake_Subsystem.StopIntakeCommand()));

*/

 // button.OnTrue(std::move(fooCommand).AndThen(std::move(barCommand).RaceWith(std::move(bazCommand))));

/*
  //Left Bumpers call for intake                                             //IntakeConstants::high_SpitSpeedConstant
  m_TopController.RightBumper().WhileTrue(intake_Subsystem.SpitIntakeCommand(IntakeConstants::high_SpitSpeedConstant));
  m_TopController.RightBumper().OnFalse(intake_Subsystem.StopIntakeCommand());
  
  //-------------------------------------------------------------------------------//



  //-------------------------------------------------------------------------------//
  //Arm positions for each Button
/*
  if(RobotContainer::mTopController.GetRawButton(1)==true){
    shooter_Subsystem.ShootSpeakerCommand(ShooterConstants::speaker_ShootSpeedConstant);
    
  }
  */

 /*
  m_TopController.A()
      .WhileTrue(shooter_Subsystem.ShootSpeakerCommand(ShooterConstants::speaker_ShootSpeedConstant););
      .WhileFalse(shooter_Subsystem.StopShooterCommand(););
*/

/*
  m_TopController.A().WhileTrue(frc2::InstantCommand([&]() -> void
                                         {shooter_Subsystem.ShootSpeakerCommand(ShooterConstants::speaker_ShootSpeedConstant);},
                                         {&shooter_Subsystem})
                                         
                        .ToPtr());
                        */
/*

  m_TopController.B().WhileTrue(frc2::InstantCommand([&]() -> void
                                         {arm_Subsystem.SetArmPosition(false, true, false);},
                                         {&arm_Subsystem})
                        .ToPtr());

   m_TopController.Y().WhileTrue(frc2::InstantCommand([&]() -> void
                                         {arm_Subsystem.SetArmPosition(true, false, false);},
                                         {&arm_Subsystem})
                        .ToPtr());
  

  m_TopController.A().OnFalse(frc2::InstantCommand([&]() -> void
                                         {arm_Subsystem.DefaultPosition();},
                                         {&arm_Subsystem})
                        .ToPtr());
  

  m_TopController.B().OnFalse(frc2::InstantCommand([&]() -> void
                                         { arm_Subsystem.DefaultPosition();},
                                         {&arm_Subsystem})
                        .ToPtr());

  m_TopController.Y().OnFalse(frc2::InstantCommand([&]() -> void
                                         { arm_Subsystem.DefaultPosition();},
                                         {&arm_Subsystem})
                        .ToPtr());
*/
  //  //-------------------------------------------------------------------------------//

}

void RobotContainer::AutonomousInit(){swerve_Subsystem.ResetEncoders();
swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));}

void RobotContainer::TeleopInit(){swerve_Subsystem.ResetEncoders();
swerve_Subsystem.ResetOdometry(frc::Pose2d(0_m, 0_m, frc::Rotation2d{0_deg}));}



std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {


  // An example command will be run in autonomous

  // // Set up config for trajectory
  // frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
  //                              AutoConstants::kMaxAcceleration);

  // // Add kinematics to ensure max speed is actually obeyed
  // config.SetKinematics(DriveConstants::kDriveKinematics);

  //   // Generate trajectory to follow.  All units in meters.
  //   auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     // Start at the origin facing the +X direction
  //     frc::Pose2d{0_m, 0_m, 0_deg},

  //     // Pass through these two interior waypoints, making an 's' curve path
  //     {frc::Translation2d{1_m, 0_m}},

  //     // End 3 meters straight ahead of where we started, facing forward
  //     frc::Pose2d{-1_m, 1_m, 0_deg},
  //     // Pass the config
  //     config);
    
  //   //Define PID controllers for tracking trajectory
  //   frc2::PIDController xController{AutoConstants::kPXController, 0, 0};
  //   frc2::PIDController yController{AutoConstants::kPYController, 0, 0};
  //   frc::ProfiledPIDController<units::radians> thetaController{
  //     AutoConstants::kPThetaController, 0, 0,
  //     AutoConstants::kThetaControllerConstraints};

  //   thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
  //                                       units::radian_t{std::numbers::pi});

  //   // Construct command to follow trajectory
  //   frc2::SwerveControllerCommand<4> swerveControllerCommand(
  //     trajectory,
  //     [this]() { return swerve_Subsystem.GetPose2d(); },
  //     DriveConstants::kDriveKinematics,
  //     xController,
  //     yController, 
  //     thetaController,
  //     [this](auto moduleStates) { swerve_Subsystem.SetModulesState(moduleStates); },
  //     {&swerve_Subsystem});                                    

  //     // Reset odometry to the starting pose of the trajectory.(WPLIB)
  //     swerve_Subsystem.ResetOdometry(trajectory.InitialPose());

  //       //Add some init and wrap-up, and return everything
  // return new frc2::SequentialCommandGroup(
  //     std::move(swerveControllerCommand),
  //     frc2::InstantCommand(
  //         [this]() { swerve_Subsystem.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));
 
  
       
  // // Set up config for trajectory
  // frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
  //                              AutoConstants::kMaxAcceleration);

  // // Add kinematics to ensure max speed is actually obeyed
  // config.SetKinematics(DriveConstants::kDriveKinematics);



   // Generate trajectory to follow.  All units in meters.

    //  frc::Trajectory trajectory =  frc::TrajectoryGenerator::GenerateTrajectory({
    //   // Start at the origin facing the +X direction
    //   frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}},

    //   // // Pass through these two interior waypoints, making an 's' curve path
    //   //  {frc::Translation2d{0.1_m, 0_m}},

    //   // End 3 meters straight ahead u where we started, facing forward
    //   frc::Pose2d{0.5_m,0_m, frc::Rotation2d{0_deg}}},

      
    //   // Pass the config
    //   config); 

    
    return TrajectoryAuto::TrajectoryAutoCommandFactory(&swerve_Subsystem, 
    frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}, 
   {frc::Translation2d(0_m, 1_m), frc::Translation2d(-1_m, 0_m)},
    frc::Pose2d{0_m, -1_m, frc::Rotation2d{200_deg}});
      
}