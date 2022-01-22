package com.wpilib;

import com.team254.lib.geometry.Rotation2d;

/**
 * Represents the state of one swerve module.
 */
@SuppressWarnings("MemberName")
public class SwerveModuleState implements Comparable<SwerveModuleState> {

  /**
   * Speed of the wheel of the module.
   */
  public double speedMetersPerSecond;

  /**
   * Angle of the module.
   */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /**
   * Constructs a SwerveModuleState with zeros for speed and angle.
   */
  public SwerveModuleState() {
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
  }

  /**
   * Compares two swerve module states. One swerve module is "greater" than the other if its speed
   * is higher than the other.
   *
   * @param o The other swerve module.
   * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
   */
  @Override
  @SuppressWarnings("ParameterName")
  public int compareTo(SwerveModuleState o) {
    return Double.compare(this.speedMetersPerSecond, o.speedMetersPerSecond);
  }
}