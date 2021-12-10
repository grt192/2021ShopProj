package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

import static frc.robot.Constants.ClawConstants.*;

public class ClawSubsystem extends SubsystemBase {

  private TalonSRX rightMotor;
  private TalonSRX leftMotor;
  private Solenoid pfft;

  // Position variables for the motors (0 is open)
  private double prevRightPos;
  private double prevLeftPos;

  // State variables for claw close/open and pneumatic on/off
  public boolean liftClaw;
  public boolean closeClaw;

  /**
   * Initializes a ClawSubsystem. Assumes the claw is in a totally open position
   * before construction.
   */
  public ClawSubsystem() {
    CommandScheduler.getInstance().registerSubsystem(this);

    // Initialize motors and pneumatic
    rightMotor = new TalonSRX(rightMotorPort);
    leftMotor = new TalonSRX(leftMotorPort);

    pfft = new Solenoid(pfftPCMPort);

    // Configure the motors for the encoders
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMotor.setSelectedSensorPosition(0);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    leftMotor.setSelectedSensorPosition(0);

    leftMotor.setInverted(InvertType.InvertMotorOutput);

    // Set initial motor position vars (for checking if motors are stalled)
    prevRightPos = 0;
    prevLeftPos = 0;

    liftClaw = false;
    closeClaw = false;
  }

  @Override
  public void periodic() {
    System.out.println("right: " + rightMotor.getSelectedSensorPosition());
    System.out.println("left: " + leftMotor.getSelectedSensorPosition());

    // Set motor powers
    if (closeClaw) {
      // Continue to power motors if claw is not closed yet
      if (!isMotorStalled(true)) {
        rightMotor.set(ControlMode.PercentOutput, -rightOpenPower);
      } else {
        rightMotor.set(ControlMode.PercentOutput, 0);
      }
      if (!isMotorStalled(false)) {
        leftMotor.set(ControlMode.PercentOutput, -leftOpenPower);
      } else {
        leftMotor.set(ControlMode.PercentOutput, 0);
      }
    } else {
      // Continue to power motors if claw is not open yet
      if (rightMotor.getSelectedSensorPosition() > 0) {
        rightMotor.set(ControlMode.PercentOutput, rightOpenPower);
      } else {
        rightMotor.set(ControlMode.PercentOutput, 0);
      }
      if (leftMotor.getSelectedSensorPosition() > 0) {
        leftMotor.set(ControlMode.PercentOutput, leftOpenPower);
      } else {
        leftMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    // Set pneumatic
    pfft.set(liftClaw);
  }

  /**
   * Checks if the claw motors are not moving (ie. if change in position is less
   * than a small delta value).
   * 
   * @return true if stalled; false if moving
   */
  public boolean isMotorStalled(boolean right) {

    boolean stalled;

    if (right) {
      double newRightPos = rightMotor.getSelectedSensorPosition();

      // If motors are barely changing position
      stalled = Math.abs(newRightPos - prevRightPos) <= stallDelta;

      prevRightPos = newRightPos;

    } else {
      double newLeftPos = leftMotor.getSelectedSensorPosition();

      // If motors are barely changing position
      stalled = Math.abs(newLeftPos - prevLeftPos) <= stallDelta;

      prevLeftPos = newLeftPos;
    }

    return stalled;
  }

  /**
   * Returns true if both motors are stalled.
   * 
   * @return true if both are stalled
   */
  public boolean areBothMotorsStalled() {
    return isMotorStalled(true) && isMotorStalled(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
