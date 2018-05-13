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

    private static Vector _v = new Vector(0, 0, 0);
    private static Vector _u = new Vector(0, 0, 0);
    private static Vector _w = new Vector(0, 0, 0);
    private static Point _p = new Point(0, 0, 0);
    public static boolean compute2CirclesIntersection(Plane cir1, double r1, Plane cir2, double r2, Point p1, Point p2){
        if (!Plane.getIntersectionLine(cir1, cir2, _v, _p)){
            return false;
        }
        _u.changeVector(cir1.p, _p);
        _v.multiply(_u.dotProduct(_v));
        _w.assignAddition(_u, _v.multiply(-1.0));
        if (_w.sqrtMagnitude() - r1 < 0.0){
            double odvesna = Math.sqrt(Math.pow(r1, 2) - _w.dotProduct(_w));
            //todo todo todo
        }
        return true;
    }
    /*
    this method computes intersection points between two circles of the same radius and with the same center
    useful when checking whether two edges of triangles on a sphere intersect
     */
    public static boolean compute2CirclesIntersection(Plane cir1, Plane cir2, double r, Point p1, Point p2){
        if (!Plane.getIntersectionLine(cir1, cir2, _v, _p)){
            return false;
        }
        _u.changeVector(cir1.p, _p);
        double d = _u.dotProduct(_v);
        _v.multiply(d);
        _w.assignAddition(_u, _v.multiply(-1.0));
        _v.makeUnit().multiply(r);
        p1.assignTranslation(cir1.p, _v);
        p2.assignTranslation(cir1.p, _v.multiply(-1.0));
        return true;
    }
}
