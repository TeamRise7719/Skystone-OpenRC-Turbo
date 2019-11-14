package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV.DogeMath;


import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV.DogeHardware.DogeHardware;

/**
 * @Author Sean Cardosi
 * @Date 11/14/19
 */

public class DogePoint implements DogeHardware {
    private double x, y, h;

    public DogePoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public DogePoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public double getH() {
        return h;
    }

    public void setH(double h) {
        this.h = h;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }
    public boolean equals(DogePoint point) {
        double x = Math.abs(getX());
        double y = Math.abs(getY());
        double x1 = Math.abs(point.getX());
        double y1 = Math.abs(point.getY());
        if ((x - x1) < 0.1 && (y - y1) < 0.1) return true;
        else return false;
    }

    public DogeVector toVector() {
        return new DogeVector(getX(), getY());
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X: " + Double.toString(getX()),
                "Y: " + Double.toString(getY())
        };
    }
}