// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivesubsystem extends SubsystemBase {
  WPI_TalonSRX lmotora = new WPI_TalonSRX(0);
  WPI_TalonSRX lmotorb = new WPI_TalonSRX(1);
  WPI_TalonSRX rmotora = new WPI_TalonSRX(2);
  WPI_TalonSRX rmotorb = new WPI_TalonSRX(3);
  MotorControllerGroup leftMotors = new MotorControllerGroup(lmotora, lmotorb);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rmotora, rmotorb);
  DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);
  /*Encoder leftEncoder = new Encoder(0, 1);
  Encoder rightEncoder = new Encoder(2, 3); */
  XboxController drivestick = new XboxController(0);


  //public double getencodermeters(){
    //return (leftEncoder.get() + -rightEncoder.get()) / 2 * kEncoderTick2Meter;
 // }
  public void setmotors(){
  
  //leftMotors.set(lmotorspeed);
  //rightMotors.set(rmotorspeed);
  rightMotors.setInverted(true);
  leftMotors.setInverted(false);

  m_drive.tankDrive(-drivestick.getLeftY() *.67, drivestick.getRightX() *.67);
  }
 // public void setMotors(double leftspeed, double rightspeed){

  // leftMotors.set(lmotorspeed);
    //rightMotors.set(rmotorspeed);
  
  public drivesubsystem() {

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
