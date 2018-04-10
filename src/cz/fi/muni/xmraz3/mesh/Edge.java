package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;

public class Edge {
    public int v1;
    public int v2;
    public Edge next;
    public Edge prev;
    public Point p1;
    public Point p2;
    //int frontFaceID = -1;
    //public Arc owner;
    // for AFM purposes, deals with sitation when there are more than one arcs
    int loopID = 0;
    public Edge(){}
    public Edge(int v1, int v2){
        this.v1 = v1;
        this.v2 = v2;
    }

    public Vector getVector(){
        return Point.subtractPoints(p2, p1);
    }

    public String toOBJString(int offset){
        return "" + (v1 + 1 + offset) + " " + (v2 + 1 + offset);
    }
}
