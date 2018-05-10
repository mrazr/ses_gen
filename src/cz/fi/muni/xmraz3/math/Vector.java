package cz.fi.muni.xmraz3.math;

/**
 * Created by radoslav on 22.11.2016.
 */
public class Vector {
    private double x;
    private double y;
    private double z;

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

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }



    public double[] getData(){ return new double[]{x, y, z};}

    public float[] getFloatData(){
        return new float[]{(float)x, (float)y, (float)z};
    }

    public Vector(Vector v){
        x = v.x;
        y = v.y;
        z = v.z;
    }

    public Vector(float[] data){
        x = data[0];
        y = data[1];
        z = data[2];
    }

    public Vector(double[] data){
        x = data[0];
        y = data[1];
        z = data[2];
    }

    public Vector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector multiply(double factor){
        x *= factor;
        y *= factor;
        z *= factor;
        return this;
    }

    public double dotProduct(Vector v2){
        double res = x*v2.x+y*v2.y+z*v2.z;
        return res;
    }

    public double sqrtMagnitude(){
        double square = dotProduct(this);
        return Math.sqrt(square);
    }

    public static Vector scaleVector(Vector v, double factor){
        Vector a = new Vector(v.x, v.y, v.z);
        return a.multiply(factor);
    }

    public Vector makeUnit(){
        double mag = this.sqrtMagnitude();
        x /= mag;
        y /= mag;
        z /= mag;
        return this;
    }

    public static Vector addVectors(Vector v1, Vector v2) {
        Vector viktor = v1;
        Vector vektor = v2;
        Vector pektor = new Vector(viktor.getX() + vektor.getX(), viktor.getY() + vektor.getY(), viktor.getZ() + vektor.getZ());
        //pektor.multiply(0.5);
        //return new Vector(v1.getX() + v2.getX(), v1.getY() + v2.getY(), v1.getZ() + v2.getZ());
        return pektor;
    }

    public static Vector subtractVectors(Vector u, Vector v){
        Vector res = Vector.addVectors(u, v.multiply(-1));
        v.multiply(-1);
        return res;
    }

    public static Vector getNormalVector(Vector u, Vector v){
        double w1 = u.y*v.z - u.z*v.y;
        double w2 = u.z*v.x - u.x*v.z;
        double w3 = u.x*v.y - u.y*v.x;
        return new Vector(w1, w2, w3);
    }

    public Vector assignNormalVectorOf(Vector u, Vector v){
        x = u.y*v.z - u.z*v.y;
        y = u.z*v.x - u.x*v.z;
        z = u.x*v.y - u.y*v.x;
        return this;
    }

    public Vector assignAddition(Vector u, Vector v){
        this.x = u.x + v.x;
        this.y = u.y + v.y;
        this.z = u.z + v.z;
        return this;
    }

    public Vector projectionOnto(Vector v){
        v.makeUnit();
        Vector proj = new Vector(v);
        proj.multiply(this.dotProduct(v));
        return proj;
    }

    public static Vector projectionOntoPlane(Vector v, Vector n){
        Vector n_ = new Vector(n);
        Vector v_ = new Vector(v);
        v_.makeUnit();
        n_.makeUnit();
        n_.multiply(n_.dotProduct(v_));
        n_.multiply(-1.0);
        return addVectors(v_, n_);
    }

    public void add(Vector v){
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
    }
    private static double[] data = new double[3];
    private static double[] ndata = new double[3];
    public Vector getPerpendicularVector(Vector in){
        int zero = -1;
        int first = -1;
        int second = -1;
        boolean twoZeroes = false;
        //double[] data = this.getData();
        data[0] = this.x;
        data[1] = this.y;
        data[2] = this.z;
        for (int i = 0; i < 3; i++){
            if (Math.abs(data[i]) < 0.0001){
                if (zero < 0) {
                    zero = i;
                } else {
                    twoZeroes = true;
                }
            } else {
                if (first < 0){
                    first = i;
                } else if (second < 0){
                    second = i;
                }
            }
        }
        if (twoZeroes){
            switch (first){
                case 0:
                    return new Vector(0, 1.0, 0);
                case 1:
                    return new Vector(1.0, 0, 0);
                case 2:
                    return new Vector(1.0, 0.0, 0.0);
            }
        }
        if (zero >= 0){
            first = zero;
        }
        double tmp = -data[first];
        //double[] ndata = new double[3];
        ndata[0] = data[0];
        ndata[1] = data[1];
        ndata[2] = data[2];
        ndata[first] = -data[second];
        ndata[second] = data[first];
        for (int i = 0; i < 3; ++i){
            if (i != first && i != second){
                ndata[i] = 0.0;
                break;
            }
        }
        //Vector v = new Vector(ndata);
        in.changeVector(ndata[0], ndata[1], ndata[2]);
        if (Math.abs(in.dotProduct(this)) > 0.0001){
            System.out.println("something wrong");
        }
        return in;
    }

    public Vector changeVector(Point p, Point q){
        x = p.x - q.x;
        y = p.y - q.y;
        z = p.z - q.z;
        return this;
    }

    public Vector changeVector(Vector v){
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        return this;
    }

    public Vector changeVector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    /*@Override
    public String toString() {
        return "Vector{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }*/
    @Override
    public String toString() {
        return "" + x + " " + y + " " + z;
    }
}
