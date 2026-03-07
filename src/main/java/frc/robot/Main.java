// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.logging.log4j.core.config.Configurator;
import org.apache.logging.log4j.core.config.builder.api.AppenderComponentBuilder;
import org.apache.logging.log4j.core.config.builder.api.ConfigurationBuilder;
import org.apache.logging.log4j.core.config.builder.api.ConfigurationBuilderFactory;
import org.apache.logging.log4j.core.config.builder.api.LayoutComponentBuilder;
import org.apache.logging.log4j.core.config.builder.impl.BuiltConfiguration;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>
   * If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    ConfigurationBuilder<BuiltConfiguration> builder = ConfigurationBuilderFactory.newConfigurationBuilder();
    builder.setStatusLevel(Constants.kLogLevel);
    builder.setConfigurationName("DefaultConfig");

    LayoutComponentBuilder layout = builder.newLayout("PatternLayout")
        .addAttribute("pattern", "%d{HH:mm} [%t] %-5level %c{1} - %msg%n%throwable");

    AppenderComponentBuilder console = builder.newAppender("Console", "CONSOLE").add(layout);
    builder.add(console);

    builder.add(builder.newRootLogger(org.apache.logging.log4j.Level.INFO).add(builder.newAppenderRef("Console")));

    Configurator.initialize(builder.build());

    RobotBase.startRobot(Robot::new);
  }
}
