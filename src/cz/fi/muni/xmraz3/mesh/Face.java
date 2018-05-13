package cz.fi.muni.xmraz3.mesh;


public class Face{
    public int a;
    public int b;
    public int c;
    //public boolean forceRefine = false;
    //public boolean valid = true;
    //public boolean divisible = true;
    public Face(int ca, int cb, int cc){
        a = ca;
        b = cb;
        c = cc;
    }

    @Override
    public String toString() {
        return "Face{" +
                "a=" + a +
                ", b=" + b +
                ", c=" + c +
                '}';
    }
}
