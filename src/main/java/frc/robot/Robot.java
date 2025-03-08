// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import frc.robot.subsystems.SwerveSubsystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public XboxController controller1;
  public XboxController controller2;

  //private MainDrive driveCommand;
  SwerveSubsystem swerve = new SwerveSubsystem();
  SparkMax elevator1;
  SparkMax elevator2;
  DigitalInput elevatorBottom;
  DigitalInput elevatorTop;

  SparkMax spin;
  CANcoder spinencoder;

  SparkMax climber;

  SparkMax grabber1;
  SparkMax grabber2;


  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
   // private double auto_start_time;
    
    controller1 = new XboxController( 0 );
    controller2 = new XboxController( 1 );

    elevator1 = new SparkMax(15, MotorType.kBrushless);
    // elevator 2 setup as inverted follower on can id 16
    elevatorBottom = new DigitalInput(0);
    elevatorTop    = new DigitalInput(1);

    spin = new SparkMax(17, MotorType.kBrushless);
    //spinencoder = new CANcoder(20);

    climber = new SparkMax(22, MotorType.kBrushless);

    grabber1 = new SparkMax(18, MotorType.kBrushless);
    grabber2 = new SparkMax(19, MotorType.kBrushless);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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

  // This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
  @Override
  public void autonomousInit() {
  //  auto_start_time = Timer.getFPGATimestamp(); 
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  //double current_time = Timer.getFPGATimestamp();
 //   double auto_time = current_time - auto_start_time;

   // swerve.drive(x, y, rotation);
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

    // This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() {

    double elevatorspeed = controller2.getRightY();
    boolean atTop = !elevatorTop.get();
    boolean atBottom = !elevatorBottom.get();
    
   /**  
    if (controller2.getBButton()){

    } else if ( ){

    } else ( ){

    }
    */

   /*  if(controller2.getleftStickAxses1 > .5 ) {
     elevator1.set(0.1);
    } else if (controller2.getleftStickAxes1 < .5) {
     elevator1.set(-0.1);
    } else(controller2.getLeftStickAxses1 = 0 ) {
    elevator1.set(0);
    }
*/
    if (atBottom && elevatorspeed > 0) {
      System.out.println("At Bottom");
      elevatorspeed = 0;
    }
    if (atTop && elevatorspeed < 0) {
      System.out.println("At Top");
      elevatorspeed = 0;
    }
    elevator1.set(-elevatorspeed);


    double spinSpeed = 0.1;


    //double spinPosition = spinencoder.getPosition().getValueAsDouble();
    //System.out.println(spinPosition);
    if (controller2.getXButton()) {
      spin.set(spinSpeed); 
    } else if (controller2.getBButton()) {
      spin.set(-spinSpeed);
    } else {
      spin.set(0);
    }
    
    double climberSpeed = 1;
    if (controller2.getAButton()) {
      climber.set(-climberSpeed);
    } else if (controller2.getYButton()) {
    climber.set(climberSpeed);
  } else{
    climber.set(0);
  }

//I suspect that the left bumper is in and the right bumper is out
    double grabberSpeed = 0.6;
    if (controller2.getLeftTriggerAxis() >0.5 ) {
      grabber1.set(grabberSpeed);
      grabber2.set(-grabberSpeed);
    } else if (controller2.getRightTriggerAxis() >0.5 ) {
      grabber1.set(-grabberSpeed);
      grabber2.set(grabberSpeed);
    } else{
      grabber1.set(0);
      grabber2.set(0);
    }

    /*
    double grabberConstant = 0.4;
    double grabberSpeed;
    if (controller2.getLeftTriggerAxis() >0.5 ) {
      grabberSpeed = grabberConstant;
    } else if (controller2.getRightTriggerAxis() >0.5 ) {
      grabberSpeed = -grabberConstant;
    } else {
     grabberSpeed = 0
    }
     
    grabber1.set(grabberSpeed);
    grabber2.set(-grabberSpeed);
    */

    double x = controller1.getLeftX();
    double y = controller1.getLeftY();
    double rotation = controller1.getRightX();

    swerve.drive(x, y, rotation);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}


