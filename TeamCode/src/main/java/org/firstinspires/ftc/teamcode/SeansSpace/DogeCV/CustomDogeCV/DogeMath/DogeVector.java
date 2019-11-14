package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV.DogeMath;

import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.CustomDogeCV.DogeHardware.DogeHardware;

/**
 * @Author Sean Cardosi
 * @Date 11/14/19
 */

public class DogeVector implements DogeHardware {
    private double x;
    private double y;
    private String name;
    public DogeVector(double x, double y, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    public DogeVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public DogePoint toPoint() {
        return new DogePoint(getX(), getY());
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public DogeVector unitVector () {
        return new DogeVector(getX()/getMagnitude(), getY()/getMagnitude());
    }

    public static DogeVector multiply(double scalar, DogeVector v) {
        return new DogeVector(v.getX() * scalar, v.getY() * scalar);
    }

    public double getMagnitude () {
        return Math.hypot(getX(), getY());
    }

    public double dotProduct(DogeVector v) {
        return (this.getX() * v.getX()) + (this.getY() * v.getY());
    }

    public double angleRad(DogeVector v) {
        return (Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
    }
    public double angleDeg(DogeVector v) {
        double deg = Math.toDegrees(Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
        if (Double.isNaN(deg)) return 0;
        return deg;
    }

    public double distanceToVector(DogeVector point) {
        return Math.hypot(point.getX() - getX(), point.getY() - getY());
    }

    public boolean equal(double radius, DogeVector v) {
        return distanceToVector(v) < radius;
    }

    public DogeVector displacement(DogeVector v) {
        return new DogeVector(v.getX() - getX(), v.getY() - getY());
    }

    public DogeVector projectOnTo(DogeVector v) {
        return multiply(dotProduct(v) / (v.getMagnitude() * v.getMagnitude()), v);
    }

    public double getDirection () {
        return Math.toDegrees(Math.atan(getY() / getX()));
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X: " + getX(),
                "Y: " + getY(),
                "Direction: " + getDirection()
        };
    }
}