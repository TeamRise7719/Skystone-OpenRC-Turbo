package org.firstinspires.ftc.teamcode.Fraser.RoadRunner;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by Sean Cardosi on 2020-01-04.
 */
@MotorType(ticksPerRev = 537.6, gearing = 19.2, maxRPM = 340, orientation = Rotation.CW)
@DeviceProperties(xmlTag="NeveRestOrbital20Gearmotor", name="NeveRest Orbital 20 Gearmotor", builtIn = false)
public interface NeveRestOrbital20Gearmotor {}
