// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class AllianceAprilTags {
    // TODO: fun fact, it's not reefscape anymore; replace tags
    public int leftCoralStation;
    public int rightCoralStation;
    public int processor;
    public int rightBarge;
    public int leftBarge;
    public int leftFrontReef;
    public int middleFrontReef;
    public int rightFrontReef;
    public int leftBackReef;
    public int middleBackReef;
    public int rightBackReef;

    public AllianceAprilTags(
        int leftCoralStation,
        int rightCoralStation,
        int processor,
        int rightBarge,
        int leftBarge,
        int leftFrontReef,
        int middleFrontReef,
        int rightFrontReef,
        int leftBackReef,
        int middleBackReef,
        int rightBackReef
    ){
        this.leftCoralStation = leftCoralStation;
        this.rightCoralStation = rightCoralStation;
        this.processor = processor;
        this.rightBarge = rightBarge;
        this.leftBarge = leftBarge;
        this.leftFrontReef = leftFrontReef;
        this.middleFrontReef = middleFrontReef;
        this.rightFrontReef = rightFrontReef;
        this.leftBackReef = leftBackReef;
        this.middleBackReef = middleBackReef;
        this.rightBackReef =rightBackReef;
    }
}