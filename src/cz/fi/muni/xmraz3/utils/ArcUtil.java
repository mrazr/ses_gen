package cz.fi.muni.xmraz3.utils;

import com.jogamp.opengl.math.Quaternion;
import cz.fi.muni.xmraz3.*;
import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.mesh.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class ArcUtil {



    public static void refineArc(Arc a, double maxLen, boolean fixedCount, int numOfSubdivisions, boolean fullCircle){
        try {
            int it = 0;
            if (a.mid == null){
                Vector v = Point.subtractPoints(a.end1, a.end2).multiply(0.5f);
                Point mid = Point.translatePoint(a.end2, v);
                Vector toMid = Point.subtractPoints(mid, a.center).makeUnit().multiply(a.radius);
                boolean useMid = false;
                if (Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
                    toMid.multiply(-1.0);
                    //useMid = true;
                }
                a.mid = Point.translatePoint(a.center, toMid);
                /*if (useMid){
                    a.vrts.add(1, a.mid);
                }*/
            }
            double angle = getAngleR(a);
            if (!fixedCount && Math.toRadians(280) - angle < 0.0){
                refineArc(a, 0, true, 2, false);
            } else if (!fixedCount && Math.PI - angle < 0.0){
                refineArc(a, 0, true, 1, false);
            }
            while ((!fixedCount && Point.subtractPoints(a.vrts.get(0), a.vrts.get(1)).sqrtMagnitude() > 1.2 * maxLen) || (fixedCount && it < numOfSubdivisions)) {
                List<Point> newVerts = new ArrayList<>();
                for (int i = 0; i < a.vrts.size() - ((fullCircle) ? 0 : 1); ++i) {
                    Point v1 = a.vrts.get(i);
                    Point v2 = (i < a.vrts.size() - 1) ? a.vrts.get(i + 1) : a.vrts.get(0);
                    Point tmp = null;
                    if (a.vrts.size() == 2){
                        if (a.mid != null) {
                            tmp = a.mid;
                        } else {
                            tmp = Point.translatePoint(a.end1, Point.subtractPoints(a.end2, a.end1).multiply(0.5f));
                            Vector toMid = Point.subtractPoints(tmp, a.center).makeUnit().multiply(a.radius);
                            if (Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
                                toMid.multiply(-1.0);
                            }
                            tmp = Point.translatePoint(a.center, toMid);
                        }
                    } else {
                        if (i < a.vrts.size() - 1) {
                            tmp = Point.translatePoint(a.vrts.get(i), Point.subtractPoints(a.vrts.get(i + 1), a.vrts.get(i)).multiply(0.5f));
                        } else {
                            tmp = Point.translatePoint(a.vrts.get(i), Point.subtractPoints(a.vrts.get(0), a.vrts.get(i)).multiply(0.5f));
                        }
                        Vector toMid = Point.subtractPoints(tmp, a.center).makeUnit().multiply(a.radius);
                        tmp = Point.translatePoint(a.center, toMid);
                    }

                    newVerts.add(a.vrts.get(i));
                    newVerts.add(tmp);
                    //tmp._id = a.owner.nextVertexID++;
                    //a.owner.vertices.add(tmp);
                    if (!fullCircle && i == a.vrts.size() - 2) {
                        newVerts.add(a.vrts.get(i + 1));
                    }
                }
                a.vrts = newVerts;
                it++;
            }
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static void refineArc(Arc a, double maxLen, boolean fixedCount, int numOfSubdivisions, boolean fullCircle, Map<Integer, Map<Integer, Integer>> edgeSplit){
        try {
            boolean convex = true;
            int it = 0;
            if (a.mid == null){
                Vector v = Point.subtractPoints(a.end1, a.end2).multiply(0.5f);
                Point mid = Point.translatePoint(a.end2, v);
                Vector toMid = Point.subtractPoints(mid, a.center).makeUnit().multiply(a.radius);
                boolean useMid = false;
                if (Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
                    toMid.multiply(-1.0);
                    //useMid = true;
                }
                a.mid = Point.translatePoint(a.center, toMid);
                /*if (useMid){
                    a.vrts.add(1, a.mid);
                }*/
            }
            double angle = getAngleR(a);
            if (!fixedCount && Math.toRadians(280) - angle < 0.0){
                refineArc(a, 0, true, 2, false, edgeSplit);
            } else if (!fixedCount && Math.PI - angle < 0.0){
                refineArc(a, 0, true, 1, false, edgeSplit);
            }
            while ((!fixedCount && Point.subtractPoints(a.vrts.get(0), a.vrts.get(1)).sqrtMagnitude() > 1.6 * maxLen) || (fixedCount && it < numOfSubdivisions)) {
                List<Point> newVerts = new ArrayList<>();
                for (int i = 0; i < a.vrts.size() - ((fullCircle) ? 0 : 1); ++i) {
                    Point v1 = a.vrts.get(i);
                    Point v2 = (i < a.vrts.size() - 1) ? a.vrts.get(i + 1) : a.vrts.get(0);
                    int sID = (v1._id > v2._id) ? v2._id : v1._id;
                    int bID = (v1._id > v2._id) ? v1._id : v2._id;
                    /*if (!edgeSplit.containsKey(sID)){
                        edgeSplit.put(sID, new TreeMap<>());
                    }*/
                    Point tmp = null;
                    if (a.vrts.size() == 2){
                        if (a.mid != null) {
                            tmp = a.mid;
                        } else {
                            tmp = Point.translatePoint(a.end1, Point.subtractPoints(a.end2, a.end1).multiply(0.5f));
                            Vector toMid = Point.subtractPoints(tmp, a.center).makeUnit().multiply(a.radius);
                            if (Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
                                toMid.multiply(-1.0);
                            }
                            tmp = Point.translatePoint(a.center, toMid);
                        }
                    } else {
                        if (i < a.vrts.size() - 1) {
                            tmp = Point.translatePoint(a.vrts.get(i), Point.subtractPoints(a.vrts.get(i + 1), a.vrts.get(i)).multiply(0.5f));
                        } else {
                            tmp = Point.translatePoint(a.vrts.get(i), Point.subtractPoints(a.vrts.get(0), a.vrts.get(i)).multiply(0.5f));
                        }
                        Vector toMid = Point.subtractPoints(tmp, a.center).makeUnit().multiply(a.radius);
                        tmp = Point.translatePoint(a.center, toMid);
                    }

                    newVerts.add(a.vrts.get(i));
                    newVerts.add(tmp);
                    tmp._id = a.owner.nextVertexID++;
                    //tmp.a = a;
                    a.owner.vertices.add(tmp);
                    //edgeSplit.get(sID).put(bID, tmp._id);
                    if (!fullCircle && i == a.vrts.size() - 2) {
                        newVerts.add(a.vrts.get(i + 1));
                    }
                }
                a.vrts = newVerts;
                it++;
            }
            for (Point v : a.vrts){
                v.arcPoint = true;
            }
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static void refineOppositeArcs(Arc a1, Arc a2, double maxlen, boolean meshRefine){
        Arc shorter = (a1.radius - a2.radius > 0.0) ? a2 : a1;
        Arc longer = (shorter == a2) ? a1 : a2;
        if (!meshRefine) {
            int currentLevel = getSubdivisionLevel(shorter);
            refineArc(shorter, maxlen, false, 0, false);
            int numOfDivs = getSubdivisionLevel(shorter) - currentLevel;
            refineArc(longer, 0, true, numOfDivs, false);
        } else {
            /*for (int i = 0; i < a1.vrts.size() - 1; ++i){
                Point v = a1.vrts.get(i);
                v._id = a1.owner.nextVertexID++;
                a1.owner.vertices.add(v);
            }
            for (int i = 0; i < a2.vrts.size() - 1; ++i){
                Point v = a2.vrts.get(i);
                v._id = a2.owner.nextVertexID++;
                a2.owner.vertices.add(v);
            }*/
            int currentLevel = getSubdivisionLevel(shorter);
            refineArc(shorter, maxlen, false, 0, false, MeshRefinement.convexEdgeSplitMap.get(shorter.owner.id));
            int numOfDivs = getSubdivisionLevel(shorter) - currentLevel;
            refineArc(longer, 0, true, numOfDivs, false, MeshRefinement.convexEdgeSplitMap.get(longer.owner.id));
            for (Point v : shorter.vrts){
                v.arc = shorter;
            }
            for (Point v : longer.vrts){
                v.arc = longer;
            }
        }
    }

    public static void buildEdges(Arc a){
        try {
            a.lines.clear();
            //a.lines.add(0, endEdge1);
            /*if (vrts.size() == 2){
                if (endEdge1 == null){
                    endEdge1 = new Edge(0, 1);
                    endEdge1.p1 = vrts.get(0);
                    endEdge1.p2 = vrts.get(1);
                    endEdge1.next = endEdge1;
                    endEdge1.prev = endEdge1;
                    endEdge2 = endEdge1;
                    endEdge1.owner = this;
                }
                a.lines.add(endEdge1);
                return;
            }*/
            a.lines.add(a.endEdge1);
            for (int i = 1; i < a.vrts.size() - ((a.circularArc) ? 1 : 2); ++i) {
                Edge e = new Edge(i, i + 1);
                e.owner = a;
                a.lines.add(e);
                e.p1 = a.vrts.get(i);
                e.p2 = a.vrts.get(i + 1);
                if (i > 0) {
                    Edge eprev = a.lines.get(i - 1);
                    eprev.next = e;
                    e.prev = eprev;
                }
            }
            a.lines.add(a.endEdge2);
            a.endEdge1.p2 = a.vrts.get(1);
            a.endEdge2.p1 = a.vrts.get(a.vrts.size() - ((a.circularArc) ? 1 : 2));
            a.endEdge2.prev = a.lines.get(a.lines.size() - 2);
            a.endEdge2.prev.next = a.endEdge2;
            //a.lines.add(a.endEdge2);
            /*a.endEdge1 = a.lines.get(0);
            endEdge2 = a.lines.get(a.lines.size() - 1);
            endEdge1.next = a.lines.get(1);
            endEdge1.p1 = a.vrts.get(0);
            endEdge1.p2 = a.vrts.get(1);
            endEdge1.v1 = 0;
            endEdge1.v2 = 1;
            a.lines.get(1).prev = endEdge1;

            endEdge2.p1 = a.vrts.get(a.vrts.size() - 2);
            endEdge2.p2 = a.vrts.get(a.vrts.size() - 1);
            endEdge2.v1 = a.vrts.size() - 2;
            endEdge2.v2 = vrts.size() - 1;
            a.lines.get(a.lines.size() - 2).next = endEdge2;
            if (this instanceof ConcavePatchArc){
                ConcavePatchArc c = (ConcavePatchArc)this;
                endEdge2.next = c.next.endEdge1;
                endEdge2.next.prev = endEdge2;
                endEdge1.prev = c.prev.endEdge2;
                endEdge1.prev.next = endEdge1;
            }*/

        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static void buildEdges(Boundary b, boolean clear){
        if (clear) {
            b.vrts.clear();
            for (Arc a : b.arcs) {
                linkArcs(b.arcs);
                a.bOwner = b;
                a.owner = b.patch;
                a.valid = true;
                for (int i = 0; i < a.vrts.size() - 1; ++i) {
                    b.vrts.add(a.vrts.get(i));
                }
            }
        }
        b.lines.clear();
        for (int i = 0; i < b.vrts.size(); ++i){
            Edge e = new Edge(i, (i == b.vrts.size() - 1) ? 0 : i + 1);
            e.p1 = b.vrts.get(i);
            e.p2 = b.vrts.get((i == b.vrts.size() - 1) ? 0 : i + 1);
            b.lines.add(e);
        }
        for (int i = 0; i < b.lines.size(); ++i){
            Edge first = b.lines.get(i);
            Edge second = b.lines.get((i == b.lines.size() - 1) ? 0 : i + 1);
            first.next = second;
            second.prev = first;
        }
    }

    public static double getAngleR(Arc a){
        double phi = Math.acos(a.toEnd1.dotProduct(a.toEnd2));
        if (Math.abs(a.toEnd1.dotProduct(a.toEnd2) + 1) < 0.001){
            return Math.PI;
        }
        if (Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
            phi = 2 * Math.PI - phi;
        }
        return phi;
    }

    public static Arc[] makeNewArc(SphericalPatch sp, SphericalPatch neighbor, Point e1, Point e2, Point mid, Point midProbe, boolean circular){
        if (!sp.neighbours.contains(neighbor)){
            sp.neighbours.add(neighbor);
        }
        Vector neighborToAtom = Point.subtractPoints(sp.sphere.center, neighbor.sphere.center);
        Vector atomToNeighbor = Vector.scaleVector(neighborToAtom, -1);
        Vector atomToEnd = Point.subtractPoints(e1, sp.sphere.center);
        Vector projection = atomToEnd.projectionOnto(atomToNeighbor);
        Point arcCenter = Point.translatePoint(sp.sphere.center, projection);
        double loopRadius = Math.sqrt(sp.sphere.radius * sp.sphere.radius - projection.dotProduct(projection));
        neighborToAtom.makeUnit();
        Point end1 = e1;
        Point end2 = e2;
        Vector e1ToMid = Point.subtractPoints(mid, e1);
        Vector e1ToE2 = Point.subtractPoints(e2, e1);
        Vector u1 = Point.subtractPoints(e1, arcCenter).makeUnit();
        Vector u2 = Point.subtractPoints(mid, arcCenter).makeUnit();
        Vector n = Vector.getNormalVector(u1, u2).makeUnit();
        if (circular){
            Vector atToE1 = Point.subtractPoints(e1, sp.sphere.center);
            Vector atToMid = Point.subtractPoints(mid, sp.sphere.center);
            Vector midway = Vector.getNormalVector(atToE1, atToMid).makeUnit().multiply(loopRadius);
            Point trueMid1 = Point.translatePoint(arcCenter, midway);
            Point trueMid2 = Point.translatePoint(arcCenter, midway.multiply(-1));
            Point midProbe1 = Point.translatePoint(trueMid1, Point.subtractPoints(trueMid1, sp.sphere.center).makeUnit().multiply(SesConfig.probeRadius));
            Point midProbe2 = Point.translatePoint(trueMid2, Point.subtractPoints(trueMid2, sp.sphere.center).makeUnit().multiply(SesConfig.probeRadius));
            Arc[] loops = new Arc[2];
            loops[0] = makeNewArc(sp, neighbor, e1, mid, trueMid1, midProbe1,false)[0];
            loops[1] = makeNewArc(sp, neighbor, mid, e1, trueMid2, midProbe2, false)[0];
            return loops;
        }
        e1ToMid.makeUnit();
        e1ToE2.makeUnit();
        Vector normal = Vector.getNormalVector(e1ToMid, e1ToE2);
        normal.makeUnit();
        Point triMid = Point.getMidPoint(e1, mid, e2);

        Vector e1ToCenter = Point.subtractPoints(sp.sphere.center, arcCenter).makeUnit();
        if (n.dotProduct(neighborToAtom) < 0){
            end1 = e2;
            end2 = e1;
        }
        if (Point.subtractPoints(end1, mid).sqrtMagnitude() + Point.subtractPoints(end2, mid).sqrtMagnitude() < 0.001){
            System.err.println("possible short loop, atom1 id: " + sp.id + " atom2Id: " + neighbor.id);
        }
        Arc[] arc = new Arc[1];
        arc[0] = new Arc(arcCenter, loopRadius);
        Arc a = arc[0];
        a.normal = Vector.scaleVector(neighborToAtom, 1).makeUnit();
        a.end1 = end1;
        a.end2 = end2;
        a.setEndPoints(a.end1, a.end2, false);
        a.endEdge1 = new Edge(0, 1);
        a.endEdge1.p1 = end1;
        a.endEdge1.p2 = mid;
        a.endEdge2 = new Edge(1, 2);
        a.endEdge2.p1 = mid;
        a.endEdge2.p2 = end2;
        a.mid = mid;

        a.endEdge1.next = a.endEdge2;
        a.endEdge2.prev = a.endEdge1;

        //a.toEnd1 = Point.subtractPoints(end1, sp.sphere.center).makeUnit();
        //a.toEnd2 = Point.subtractPoints(end2, sp.sphere.center).makeUnit();

        a.vrts.add(end1);
        //a.vrts.add(mid);
        a.vrts.add(end2);
        //end1.isShared = end2.isShared = mid.isShared = true;
        a.owner = sp;
        sp.arcs.add(a);
        a.midProbe = midProbe;
        return arc;
    }

    public static void linkArcs(SphericalPatch sp) {
        try {
            List<Arc> queue = new ArrayList<>(sp.arcs);
            boolean setValid = true;
            while (queue.size() > 0) {
                ArrayList<Arc> newB = new ArrayList<>();
                Arc l = queue.get(0);
                newB.add(l);
                queue.remove(l);
                //Vector loopEnd = l.toEnd1;
                //Vector pivot = l.toEnd2;
                Point loopEnd = l.end1;
                Point pivot = l.end2;
                int i = 0;
                int iterator = 0;
                //while (Math.abs(loopEnd.dotProduct(pivot) - 1) >= 0.00001) {
                while (Point.subtractPoints(loopEnd, pivot).sqrtMagnitude() > Surface.scaleFactor * 0.0002) {
                    if (iterator > sp.arcs.size() + 10) {
                        System.err.println("Cycle detected for atom id:" + sp.id);
                        System.out.println("Iterator: " + iterator);
                        System.out.println("Queue size: " + queue.size());
                        //Main.convexPatches.remove(sp);
                        sp.valid = false;
                        return;
                    }
                    iterator++;
                    if (queue.size() == 0) {
                        System.err.println("Atom " + sp.id);
                        setValid = false;
                        break;
                    }
                    Arc lop = queue.get(i);
                    //Vector vLop = lop.toEnd1;
                    Point pLop = lop.end1;
                    if (Point.subtractPoints(pLop, pivot).sqrtMagnitude() < Surface.scaleFactor * 0.0002) {
                        boolean betterCand = false;
                        Arc tmp = lop;
                        for (int j = i + 1; j < queue.size(); ++j) {
                            Arc tL = queue.get(j);
                            if (Point.subtractPoints(l.end2, tL.end1).sqrtMagnitude() < Point.subtractPoints(l.end2, lop.end1).sqrtMagnitude()) {
                                lop = tL;
                                betterCand = true;
                                //System.err.println("Found a better candidate: " + this.id);
                            }
                        }
                        newB.add(lop);
                        queue.remove(lop);
                        //pivot = lop.toEnd2;
                        pivot = lop.end2;
                        lop.end1 = l.end2;
                        lop.lines.get(0).p1 = l.end2;
                        lop.vrts.remove(0);
                        lop.vrts.add(0, l.end2);

                        l.endEdge2.next = lop.endEdge1;
                        lop.endEdge1.prev = l.endEdge2;
                        l = lop;
                        iterator = 0;
                    } else {
                        i++;
                    }
                    if (i >= queue.size()) {
                        i = 0;
                        //completeBoundary = false;
                        //System.err.println("atom id: " + this.id + " incomplete boundary");
                        //return;
                    }
                }
                if (setValid) {
                    newB.get(0).end1 = l.end2;
                    newB.get(0).lines.get(0).p1 = l.end2;
                    newB.get(0).vrts.remove(0);
                    newB.get(0).vrts.add(0, l.end2);
                    newB.get(0).endEdge1.prev = l.endEdge2;
                    l.endEdge2.next = newB.get(0).endEdge1;
                    Boundary b = new Boundary();
                    b.patch = sp;
                    b.arcs = newB;
                    sp.boundaries.add(b);
                    ArcUtil.buildEdges(b, true);
                    for (Point v : b.vrts){
                        v._id = sp.nextVertexID++;
                        sp.vertices.add(v);
                    }
                } else {
                    sp.valid = false;
                }
            }
            /*int nextIdx = 0;
            for (Boundary b : sp.boundaries){
                for (Point v : b.vrts){
                    v._id = nextIdx++;
                    sp.vertices.add(v);
                }
            }
            sp.nextVertexID = nextIdx;*/
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void reverseArc(Arc a, boolean s){
        for (int i = a.vrts.size() - 1; i >= 0; --i){
            a.vrts.add(a.vrts.get(i));
        }
        int size = a.vrts.size() / 2;
        for (int i = 0; i < size; ++i){
            a.vrts.remove(0);
        }
        if (!s){
            return;
        }
        a.end1 = a.vrts.get(0);
        a.end2 = a.vrts.get(a.vrts.size() - 1);
        a.toEnd1 = Point.subtractPoints(a.end1, a.center).makeUnit();
        a.toEnd2 = Point.subtractPoints(a.end2, a.center).makeUnit();
    }

    public static boolean checkIfNested(Boundary b1, Boundary b2){
        return (checkForOwnership(b1, b2) && checkForOwnership(b2, b1));
    }

    private static boolean checkForOwnership(Boundary b1, Boundary b2){
        try {
            Point p1 = b1.arcs.get(0).end1;
            Vector v1 = null;
            Vector v2 = null;
            if (b1.arcs.size() < 2){
                return false;
            }
            for (int i = 1; i < b1.arcs.size(); ++i) {
                Arc l = b1.arcs.get(i);
                v1 = Point.subtractPoints(l.end2, p1).makeUnit();
                v2 = Point.subtractPoints(l.end1, p1).makeUnit();
                if (Math.abs(v1.dotProduct(v2) - 1.0) > 0.01) {
                    break;
                }
            }
            Vector n = Vector.getNormalVector(v1, v2).makeUnit().multiply(-1.0);
            double maxY = 42000.0;
            for (Arc l : b1.arcs) {
                Vector u = Point.subtractPoints(l.end2, b1.patch.sphere.center);
                double dot = u.dotProduct(n);
                if (dot < maxY) {
                    maxY = dot;
                }
            }
            boolean isInside = true;
            for (Arc l : b2.arcs) {
                Vector u = Point.subtractPoints(l.end2, b2.patch.sphere.center);
                if (u.dotProduct(n) < maxY) {
                    isInside = false;
                    break;
                }
            }
            return isInside;
        } catch (Exception e){
            e.printStackTrace();
        }
        return false;
    }

    public static List<Point> generateCircArc(Point start, Point end, Point center, double radius, int n, boolean overPI){
        Vector tostart = Point.subtractPoints(start, center).makeUnit();
        Vector toend = Point.subtractPoints(end, center).makeUnit();
        Vector axis = Vector.getNormalVector(toend, tostart).makeUnit();
        /*if (tostart.dotProduct(toend) < 0.0){
            axis.multiply(-1);
        }*/

        double angle = Math.acos(toend.dotProduct(tostart));
        if (overPI){
            angle = 2 * Math.PI - angle;
            axis.multiply(-1);
        }
        if (angle - Math.PI >= 0.0){
            System.out.println(" ");
        }
        angle = angle / n;
        List<Point> vrts = new ArrayList<>();
        vrts.add(start);
        if (n > 1) {
            for (int i = 1; i < n; ++i) {
                Quaternion qRot = new Quaternion(0, 0, 0, 0);
                qRot.setFromAngleNormalAxis((float) (-i * angle), axis.getFloatData());
                float[] v = new float[3];
                v = qRot.rotateVector(v, 0, tostart.getFloatData(), 0);
                Vector u = new Vector(v);
                u.makeUnit().multiply(radius);
                vrts.add(Point.translatePoint(center, u));
            }
        }
        vrts.add(end);
        return vrts;
    }

    public static void replaceMiddleVertex(Arc a, Point newMid){
        int idx = (a.vrts.size() - 1) / 2;
        a.vrts.remove(idx);
        a.vrts.add(idx, newMid);
        a.mid = newMid;
    }

    private static final double PLANE_EPS = 0.001;
    public static Arc findContainingArc(Point p, Plane circle, SphericalPatch sp, Arc exclude){
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (a == exclude){
                    continue;
                }
                Plane plane = new Plane(a.center, a.normal);
                if (Math.abs(plane.checkPointLocation(p)) > PLANE_EPS){
                    continue;
                }
                if (Point.distance(p, a.end2) < PLANE_EPS){
                    if (circle.checkPointLocation(a.end1) > 0.0 && circle.distanceFromPlane(a.end1) > PLANE_EPS){
                        return a;
                    }
                    if (circle.checkPointLocation(a.next.end2) > 0.0){
                        return a.next;
                    }
                }
                if (Point.distance(p, a.end1) < PLANE_EPS){
                    if (circle.checkPointLocation(a.end2) > 0.0 && circle.distanceFromPlane(a.end2) > PLANE_EPS){
                        return a;
                    }
                    if (circle.checkPointLocation(a.prev.end1) > 0.0){
                        return a.prev;
                    }
                }
                if (Math.abs(circle.checkPointLocation(a.end2)) < PLANE_EPS && Math.abs(circle.checkPointLocation(a.end1)) < PLANE_EPS){
                    if (Point.distance(p, a.end2) < PLANE_EPS){
                        return a.next;
                    } else if (Point.distance(p, a.end1) < PLANE_EPS){
                        return a.prev;
                    }
                }
                if (Math.abs(plane.checkPointLocation(p)) < PLANE_EPS && Point.distance(p, a.end1) < PLANE_EPS){
                    if (circle.checkPointLocation(a.end2) < 0.0){
                        return a.prev;
                    } else {
                        return a;
                    }
                }
                if (Math.abs(plane.checkPointLocation(p)) < PLANE_EPS && a.isInside(p)){
                    return a;
                }
            }
        }
        return null;
    }

    public static Point findClosestPointOnCircle(List<Point> points, Point start, boolean includeStart, Point center, Vector normal, boolean next){
        double angle = 2 * Math.PI;
        Point closest = null;
        Vector toStart = Point.subtractPoints(start, center).makeUnit();
        for (Point p : points){
            if (includeStart && Point.distance(start, p) < 0.001){
                return p;
            }
            Vector toP = Point.subtractPoints(p, center).makeUnit();
            double alpha = (Vector.getNormalVector(toStart, toP).multiply(next ? 1 : -1).dotProduct(normal) > 0.0) ? Math.acos(toStart.dotProduct(toP)) : (2 * Math.PI - Math.acos(toStart.dotProduct(toP)));
            if (angle - alpha > 0.0){
                angle = alpha;
                closest = p;
            }
        }
        return closest;
    }

    public static void linkArcs(List<Arc> ordered){
        for (int i = 0; i < ordered.size(); ++i){
            Arc curr = ordered.get(i);
            Arc next = (i == ordered.size() - 1) ? ordered.get(0) : ordered.get(i + 1);
            curr.next = next;
            next.prev = curr;
        }
    }

    public static int getOrder(Arc a, Point p, Point q){
        Vector pV = Point.subtractPoints(p, a.center).makeUnit();
        Vector qV = Point.subtractPoints(q, a.center).makeUnit();
        double phi1 = Math.acos(pV.dotProduct(a.toEnd1));
        double phi2 = Math.acos(qV.dotProduct(a.toEnd1));
        phi1 = (Vector.getNormalVector(a.toEnd1, pV).makeUnit().dotProduct(a.normal) > 0.0) ? phi1 : 2 * Math.PI - phi1;
        phi2 = (Vector.getNormalVector(a.toEnd1, qV).makeUnit().dotProduct(a.normal) > 0.0) ? phi2 : 2 * Math.PI - phi2;
        return (phi1 - phi2 < 0.0) ? -1 : 1;
    }

    public static Arc dbgCloneArc(Arc a){
        Arc na = new Arc(a.center, a.radius);
        na.setEndPoints(a.end1, a.end2, false);
        na.mid = a.mid;
        na.setNormal(a.normal);
        na.vrts.addAll(a.vrts);
        return na;
    }

    /*
        copies the arc a into new one, cloning its vertices instead of just sharing them with the original arc
     */
    public static Arc cloneArc(Arc a){
        Arc newA = new Arc(a.center, a.radius);
        a.vrts.forEach(v -> newA.vrts.add(new Point(v)));
        newA.setNormal(a.normal);
        newA.setEndPoints(newA.vrts.get(0), newA.vrts.get(newA.vrts.size() - 1), false);
        return newA;
    }

    public static Boundary generateCircularBoundary(Plane circle, double radius){
        Vector perp = circle.v.getPerpendicularVector().makeUnit().multiply(radius);
        Vector perp2 = Vector.getNormalVector(circle.v, perp).makeUnit().multiply(radius);
        Arc a1 = new Arc(circle.p, radius);
        Point p1 = Point.translatePoint(circle.p, perp);
        Point mid1 = Point.translatePoint(circle.p, perp2);
        perp2.multiply(-1);
        Point mid2 = Point.translatePoint(circle.p, perp2);
        perp.multiply(-1);
        Point p2 = Point.translatePoint(circle.p, perp);
        a1.setEndPoints(p1, p2, false);
        a1.setNormal(circle.v);
        a1.mid = mid1;
        a1.vrts.add(p1);
        a1.vrts.add(mid1);
        a1.vrts.add(p2);
        //ArcUtil.refineArc(a1, 0, true, 1, false);
        ArcUtil.refineArc(a1, Surface.maxEdgeLen, false, 0, false);
        Arc a2 = new Arc(circle.p, radius);
        a2.setEndPoints(p2, p1, false);
        a2.setNormal(circle.v);
        a2.mid = mid2;
        a2.vrts.add(p2);
        a2.vrts.add(mid2);
        a2.vrts.add(p1);
        //ArcUtil.refineArc(a2, 0, true, 1, false);
        ArcUtil.refineArc(a2, Surface.maxEdgeLen, false, 0, false);
        Boundary b = new Boundary();
        b.arcs.add(a1);
        b.arcs.add(a2);
        ArcUtil.buildEdges(b, true);
        a1.halfCircle = true;
        a2.halfCircle = true;
        return b;
    }

    public static int getSubdivisionLevel(Arc a){
        return (int)(Math.log10(a.vrts.size() - 1) / Math.log10(2));
    }

    public static void markShared(Arc a){
        a.vrts.stream().forEach(v -> v.isShared = true);
    }

    public static void generateEdgeSplits(Arc a, SphericalPatch sp){
        Map<Integer, Map<Integer, Integer>> edgeSplit = (sp.convexPatch) ? MeshRefinement.convexEdgeSplitMap.get(sp.id) : MeshRefinement.concaveEdgeSplitMap.get(sp.id);
        generateEdgeSplits(0, a.vrts.size() - 1, a, edgeSplit);
    }

    private static void generateEdgeSplits(int start, int count, Arc a, Map<Integer, Map<Integer, Integer>> splits){
        if (count == 1){
            return;
        }
        Point v1 = a.vrts.get(start);
        Point v2 = a.vrts.get(start + count);
        Point v3 = a.vrts.get(start + count / 2);
        int sID = (v1._id > v2._id) ? v2._id : v1._id;
        int bID = (v1._id > v2._id) ? v1._id : v2._id;
        if (!splits.containsKey(sID)){
            splits.put(sID, new TreeMap<>());
        }
        Map<Integer, Integer> map = splits.get(sID);
        map.put(bID, v3._id);
        generateEdgeSplits(start, count / 2, a, splits);
        generateEdgeSplits(start + count / 2, count / 2, a, splits);
    }

    public static Arc getCommonArc(Point p1, Point p2){
        if (!p1.arcPoint || !p2.arcPoint){
            return null;
        }
        if ((p1 == p1.arc.end1 || p1 == p1.arc.end2) && (p2 == p2.arc.end1 || p2 == p2.arc.end2)){
            Boundary b = p1.arc.bOwner;
            for (Arc a : b.arcs){
                if ((p1 == a.end1 && p2 == a.end2) || (p1 == a.end2 && p2 == a.end1)){
                    //System.out.println("found the arc of " + a.vrts.size() + " vrts");
                    return a;
                }
            }
            return null;
        }
        if (p1 == p1.arc.end1 || p1 == p1.arc.end2){
            if (p1 == p2.arc.end1 || p1 == p2.arc.end2) {
                return p2.arc;
            }
            return null;
        }
        if (p2 == p2.arc.end1 || p2 == p2.arc.end2){
            if (p2 == p1.arc.end1 || p2 == p1.arc.end2) {
                return p1.arc;
            }
            return null;
        }
        if (p1.arc == p2.arc){
            return p1.arc;
        } else {
            return null;
        }
    }
}
