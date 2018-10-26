package org.usfirst.frc.team5638.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.constants;
import org.usfirst.frc.team5638.robot.Robot;
import org.usfirst.frc.team5638.robot.RobotMap;
import org.usfirst.frc.team5638.robot.commands.DriveCom;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import jaci.pathfinder.Waypoint;

public class DriveTrain extends Subsystem implements PIDOutput {

  private final TalonSRX r1 = RobotMap.r1;
  private final TalonSRX l1 = RobotMap.l1;
  private final DifferentialDrive driveTrain = RobotMap.dt;
  private final DoubleSolenoid shift = RobotMap.shift;
  private final double a = RobotMap.a;
  private final AHRS gyro = RobotMap.gyro;
  PIDController heading;
  double RotateToAngleRate;
  boolean RotateToAngle = false;
  double currentRotationRate;

  private final XboxController x0 = Robot.OI.x0;


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveCom());
  }

  public void drive() {
    double steer = x0.getRawAxis(0);
    double a = x0.getRawAxis(3);
    double b = x0.getRawAxis(2);

    double throttle = a - b;

    driveTrain.arcadeDrive(throttle, steer);
  }

  public void stop() {
    driveTrain.arcadeDrive(0, 0);
  }

  public void shiftHI() {
    shift.set(Value.kForward);
  }

  public void shiftLO() {
    shift.set(Value.kReverse);
  }

  public void angle(double angle){
    gyro.reset();

    heading = new PIDController(constants.dtkP, constants.dtkI, constants.dtkD, constants.dtkF, Robot.gyro, this, constants.t);
    heading.setInputRange(-180.0f, 180.0f);
    heading.setOutputRange(-1.0, 1.0);
    heading.setAbsoluteTolerance(constants.kToleranceDegrees);
    heading.setContinuous(true);

    if(a != angle){
      heading.enable();
      currentRotationRate = RotateToAngleRate;
    }
  }

  @Override
  public void pidWrite(double output) {
    RotateToAngleRate = output;
  }
}