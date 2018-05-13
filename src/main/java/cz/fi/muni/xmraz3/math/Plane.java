package cz.fi.muni.xmraz3.math;

import cz.fi.muni.xmraz3.mesh.AdvancingFrontMethod;

public class Plane {
    public Point p;
    public Vector v;
    double d;

    public Plane(Point p, Vector v1, Vector v2){
        this(p, Vector.getNormalVector(v1, v2).makeUnit());
    }

    public Plane(Point p, Vector v){
        this.p = p;
        this.v = v.makeUnit();
        d = - (p.x * v.getX() + p.y * v.getY() + p.z * v.getZ());
    }

    public double checkPointLocation(Point q){
        return v.getX() * q.x + v.getY() * q.y + v.getZ() * q.z + d;
    }

    public Vector getIntersectionVector(Plane pi){
        if (Math.abs(pi.v.dotProduct(this.v) - 1) > 0.00001){
            return Vector.getNormalVector(this.v, pi.v).makeUnit();
        }
        return null;
    }

    public Vector getIntersectionVector(Plane pi, Vector result){
        if (Math.abs(pi.v.dotProduct(this.v) - 1) > 0.00001){
            //return Vector.getNormalVector(this.v, pi.v).makeUnit();
            return result.assignNormalVectorOf(this.v, pi.v).makeUnit();
        }
        return null;
    }

    public boolean assignIntersectionVectorTo(Vector v, Plane pi){
        if (Math.abs(pi.v.dotProduct(this.v) - 1) > 0.00001){
            v.assignNormalVectorOf(this.v, pi.v).makeUnit();
            return true;
        }
        return false;
    }
    private static Vector v1 = new Vector(0, 0, 0);
    private static Vector v2 = new Vector(0, 0, 0);
    private static Vector v3 = new Vector(0, 0, 0);
    private static Vector dir = new Vector(0, 0, 0);
    private static Plane r = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));
    public static boolean getIntersectionLine(Plane p1, Plane p2, Vector vInt, Point pInt){
        //Vector dir = p1.getIntersectionVector(p2);
        p1.getIntersectionVector(p2, dir);
        if (dir == null){
            //System.out.println("NULL vector");
            return false;
        }
        //double det = dir.dotProduct(dir);
        double det = AdvancingFrontMethod.determinant(p1.v, p2.v, dir);
        //Plane r = new Plane(new Point(0,0,0), dir);
        r.redefine(r.p, dir);

        /*if (Math.abs(det) > 0.01){
            Vector cross1 = Vector.getNormalVector(dir, p2.v).makeUnit();
            Vector cross2 = Vector.getNormalVector(p1.v, dir).makeUnit();
            pInt.x = (cross1.getX() * p1.d + cross2.getX() * p2.d) / det;
            pInt.y = (cross1.getY() * p1.d + cross2.getY() * p2.d) / det;
            pInt.z = (cross1.getZ() * p1.d + cross2.getZ() * p2.d) / det;
            vInt.setX(dir.getX());
            vInt.setY(dir.getY());
            vInt.setZ(dir.getZ());
            return true;
        }*/
        if (Math.abs(det) > 0.001){
            //Vector cross1 = Vector.getNormalVector(p2.v, r.v);
            //Vector cross2 = Vector.getNormalVector(r.v, p1.v);
            //Vector cross3 = Vector.getNormalVector(p1.v, p2.v);
            v1.assignNormalVectorOf(p2.v, r.v);
            v2.assignNormalVectorOf(r.v, p1.v);
            v3.assignNormalVectorOf(p1.v, p2.v);
            pInt.x = (v1.getX() * (-p1.d) + v2.getX() * (-p2.d) + v3.getX() * (-r.d)) / det;
            pInt.y = (v1.getY() * (-p1.d) + v2.getY() * (-p2.d) + v3.getY() * (-r.d)) / det;
            pInt.z = (v1.getZ() * (-p1.d) + v2.getZ() * (-p2.d) + v3.getZ() * (-r.d)) / det;
            vInt.setX(dir.getX());
            vInt.setY(dir.getY());
            vInt.setZ(dir.getZ());
            return true;
        }
        return false;
    }

    public Point pointProjection(Point q){
        Vector qMp = Point.subtractPoints(q, p);
        Vector n = Vector.scaleVector(v, qMp.dotProduct(v));
        return Point.translatePoint(q, n.multiply(-1));
    }

    public double distanceFromPlane(Point p){
        Vector n = new Vector(v);
        n.multiply(v.dotProduct(Point.subtractPoints(p, this.p)));
        return n.sqrtMagnitude();
    }

    public void changePlaneOrientation(Vector u){
        //this.v = v.makeUnit();
        this.v.changeVector(u).makeUnit();
        d = - (p.x * v.getX() + p.y * v.getY() + p.z * v.getZ());
    }

    public Plane redefine(Point p, Vector v){
        //this.p = p;
        this.p.x = p.x;
        this.p.y = p.y;
        this.p.z = p.z;
        this.changePlaneOrientation(v);
        return this;
    }

    public Plane redefine(Point p, Vector v, Vector u){
        redefine(p, this.v.assignNormalVectorOf(v, u));
        return this;
    }

    public boolean isIdenticalWith(Plane plane){
        if (Math.abs(this.checkPointLocation(plane.p)) > 0.01d){
            return false;
        }
        if (Math.abs(Math.abs(this.v.dotProduct(plane.v)) - 1) > 0.001){
            return false;
        }
        return true;
    }
}
