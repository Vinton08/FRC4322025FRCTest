// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ArrayBuilder {
  /**
   * Builds an array of `SwerveModuleState` objects.
   *
   * @param size The number of elements in the array.
   * @return An initialized array of `SwerveModuleState` objects.
   */
  public static SwerveModuleState[] buildSwerveModuleState(int size) {
    if (size <= 0) {
      throw new IllegalArgumentException("Size must be positive");
    }
    SwerveModuleState[] moduleStates = new SwerveModuleState[size];
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = new SwerveModuleState();
    }
    return moduleStates;
  }

  /**
   * Builds an array of `SwerveModulePosition` objects.
   *
   * @param size The number of elements in the array.
   * @return An initialized array of `SwerveModulePosition` objects.
   */
  public static SwerveModulePosition[] buildSwerveModulePosition(int size) {
    if (size <= 0) {
      throw new IllegalArgumentException("Size must be positive");
    }
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[size];
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }
    return modulePositions;
  }
}
