package frc.robot.util;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Edited by Team 178 for visible previous value and swerve angular filter

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class RateLimiter {
  private final double m_positiveRateLimit;
  private final double m_negativeRateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new RateLimiter with the given positive and negative rate limits and initial
   * value.
   *
   * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
   *     second. This is expected to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
   *     second. This is expected to be negative.
   * @param initialValue The initial value of the input.
   */
  public RateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
    m_positiveRateLimit = positiveRateLimit;
    m_negativeRateLimit = negativeRateLimit;
    prevVal = initialValue;
    prevTime = MathSharedStore.getTimestamp();
  }

  /**
   * Creates a new RateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public RateLimiter(double rateLimit) {
    this(rateLimit, -rateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        MathUtil.clamp(
            input - prevVal,
            m_negativeRateLimit * elapsedTime,
            m_positiveRateLimit * elapsedTime);
    prevTime = currentTime;
    return prevVal;
  }

  /**
   * ADDED BY TEAM 178. Filters the angular directional input in radians to limit its slew rate with looping values from -pi to pi.
   * Use when inputs are angle positions, not rotation speeds.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double angleCalculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        MathUtil.clamp(
            SwerveDrive.swerveAngleDifference(input, prevVal),
            m_negativeRateLimit * elapsedTime,
            m_positiveRateLimit * elapsedTime);
    if(Math.abs(prevVal) > Math.PI) {
        prevVal -= Math.signum(prevVal) * 2 * Math.PI;
    }
    prevTime = currentTime;
    return prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = MathSharedStore.getTimestamp();
  }

  public double getPrevVal() {
      return prevVal;
  }
}
