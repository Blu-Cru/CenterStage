package org.firstinspires.ftc.teamcode.blucru.common.util;

public class Point2d {
    public double x;
    public double y;

    public Point2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point2d(Point2d p) {
        this.x = p.x;
        this.y = p.y;
    }

    public Point2d() {
        this(0, 0);
    }

    public static Point2d polar(double r, double theta) {
        return new Point2d(r * Math.cos(theta), r * Math.sin(theta));
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void set(Point2d p) {
        this.x = p.x;
        this.y = p.y;
    }

    public void plus(Point2d p) {
        this.x += p.x;
        this.y += p.y;
    }

    public void minus(Point2d p) {
        this.x -= p.x;
        this.y -= p.y;
    }

    public void times(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
    }

    public void divide(double scalar) {
        this.x /= scalar;
        this.y /= scalar;
    }

    public double distance(Point2d p) {
        return Math.sqrt(Math.pow(this.x - p.x, 2) + Math.pow(this.y - p.y, 2));
    }

    public double magnitude() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public void normalize() {
        double mag = magnitude();
        if(mag != 0) {
            divide(mag);
        }
    }

    public Point2d normalized() {
        Point2d p = new Point2d(this);
        p.normalize();
        return p;
    }

    public Point2d copy() {
        return new Point2d(this);
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
