// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class PIDUtil {
    private final PIDController pid;
    private String tableName;

    public PIDUtil(PIDController pid, String tableName) {
        this.pid = pid;
        this.tableName = tableName;

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable(tableName);
        NetworkTableEntry pEntry = table.getEntry("p");
        NetworkTableEntry iEntry = table.getEntry("i");
        NetworkTableEntry dEntry = table.getEntry("d");
        NetworkTableEntry targetEntry = table.getEntry("setpoint");
        pEntry.setNumber(pid.getP());
        iEntry.setNumber(pid.getI());
        dEntry.setNumber(pid.getD());

        targetEntry.setNumber(pid.getSetpoint());
    }

    public void update() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable(tableName);
        NetworkTableEntry pEntry = table.getEntry("p");
        NetworkTableEntry iEntry = table.getEntry("i");
        NetworkTableEntry dEntry = table.getEntry("d");
        NetworkTableEntry targetEntry = table.getEntry("setpoint");
        if (pEntry.getNumber(0).doubleValue() != pid.getP()) {
            pid.setP(pEntry.getNumber(0).doubleValue());
        }
        if (iEntry.getNumber(0).doubleValue() != pid.getI()) {
            pid.setI(iEntry.getNumber(0).doubleValue());
        }
        if (dEntry.getNumber(0).doubleValue() != pid.getD()) {
            pid.setD(dEntry.getNumber(0).doubleValue());
        }
        if (targetEntry.getNumber(0).doubleValue() != pid.getSetpoint()) {
            pid.setSetpoint(targetEntry.getNumber(0).doubleValue());
        }
    }

    public double calculate(double value) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable(tableName);
        NetworkTableEntry valueEntry = table.getEntry("value");
        valueEntry.setNumber(value);
        double result = pid.calculate(value);
        NetworkTableEntry resultEntry = table.getEntry("result");
        resultEntry.setNumber(result);
        return result;
    }

    public void setSetpoint(double target) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable(tableName);
        NetworkTableEntry targetEntry = table.getEntry("setpoint");
        targetEntry.setNumber(target);
        pid.setSetpoint(target);
    }
}
