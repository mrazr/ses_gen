package cz.fi.muni.xmraz3.math;

public class Sphere {
    public Point center;
    public double radius;
    private static Vector v = new Vector(0, 0, 0);
    public Sphere(Point center, double radius){
        this.center = center;
        this.radius = radius;
    }

    public static Point getContactPoint(Sphere s1, Sphere s2){
        //Vector v = Point.subtractPoints(s1.center, s2.center).makeUnit();
        v.changeVector(s1.center, s2.center).makeUnit();
        v.multiply(s2.radius);
        return Point.translatePoint(s2.center, v);
    }
}
