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


    private static Vector v = new Vector(0, 0, 0);
    private static Point mid = new Point(0, 0, 0);
    private static Point temp = new Point(0, 0, 0);
    private static Vector n = new Vector(0, 0, 0);
    private static Vector u = new Vector(0, 0, 0);
    private static List<Point> currVrts = new ArrayList<>(17);
    private static List<Point> newVrts = new ArrayList<>(17);
    public static void refineArc(Arc a, double maxLen, boolean fixedCount, int numOfSubdivisions, boolean fullCircle){
        try {
            int it = 0;
            //if (a.mid == null){
            //    v.changeVector(a.end1, a.end2).multiply(0.5f);
            //    mid.assignTranslation(a.end2, v);
            //    v.changeVector(mid, a.center).makeUnit().multiply(a.radius);
            //    if (n.assignNormalVectorOf(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){//Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
            //        v.multiply(-1.0);
            //    }
            //    a.mid = Point.translatePoint(a.center, v);
            //}
            double angle = getAngleR(a);
            if (!fixedCount && Math.toRadians(280) - angle < 0.0){
                refineArc(a, 0, true, 2, false);
            } else if (!fixedCount && Math.PI - angle < 0.0){
                refineArc(a, 0, true, 1, false);
            }
            currVrts.clear();
            newVrts.clear();
            currVrts.addAll(a.vrts);
            while ((!fixedCount && Point.distance(currVrts.get(0), currVrts.get(1)) > Surface.refineFac * maxLen) || (fixedCount && it < numOfSubdivisions)) {
                //List<Point> newVerts = new ArrayList<>();
                for (int i = 0; i < currVrts.size() - ((fullCircle) ? 0 : 1); ++i) {
                    Point tmp = null;
                    if (currVrts.size() == 2){
                        if (false){//a.mid != null) {
                            //tmp = a.mid;
                        } else {
                            if (Math.abs(Math.abs(a.toEnd1.dotProduct(a.toEnd2)) - 1.0) < 0.001){
                                v.assignNormalVectorOf(a.normal, a.toEnd1).makeUnit().multiply(a.radius);
                            } else {
                                tmp = temp.assignTranslation(a.end1, v.changeVector(a.end2, a.end1).multiply(0.5f));
                                v.changeVector(tmp, a.center).makeUnit().multiply(a.radius);
                                if (n.assignNormalVectorOf(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0) {//Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
                                    v.multiply(-1.0);
                                }
                            }
                            tmp = Point.translatePoint(a.center, v);
                        }
                    } else {
                        if (i < currVrts.size() - 1) {
                            tmp = temp.assignTranslation(currVrts.get(i), v.changeVector(currVrts.get(i + 1), currVrts.get(i)).multiply(0.5f));
                        } else {
                            tmp = temp.assignTranslation(currVrts.get(i), v.changeVector(currVrts.get(0), currVrts.get(i)).multiply(0.5f));
                        }
                        v.changeVector(tmp, a.center).makeUnit().multiply(a.radius);
                        tmp = Point.translatePoint(a.center, v);
                    }

                    newVrts.add(currVrts.get(i));
                    newVrts.add(tmp);
                    if (!fullCircle && i == currVrts.size() - 2) {
                        newVrts.add(currVrts.get(i + 1));
                    }
                }
                currVrts.clear();
                currVrts.addAll(newVrts);
                newVrts.clear();
                //a.vrts = newVerts;
                it++;
            }
            a.vrts.clear();
            a.vrts.addAll(currVrts);
            //if (a.baseSubdivision < 0){
            //    a.baseSubdivision = getSubdivisionLevel(a);
            //}
        } catch (Exception e){
            e.printStackTrace();
        }
    }

//    public static void refineArc(Arc a, double maxLen, boolean fixedCount, int numOfSubdivisions, boolean fullCircle, Map<Integer, Map<Integer, Integer>> edgeSplit){
//        try {
//            int it = 0;
//            if (a.mid == null){
//                v.changeVector(a.end1, a.end2).multiply(0.5f);
//                mid.assignTranslation(a.end2, v);
//                v.changeVector(mid, a.center).makeUnit().multiply(a.radius);
//                if (n.assignNormalVectorOf(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){//Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
//                    v.multiply(-1.0);
//                }
//                a.mid = Point.translatePoint(a.center, v);
//            }
//            double angle = getAngleR(a);
//            if (!fixedCount && Math.toRadians(280) - angle < 0.0){
//                refineArc(a, 0, true, 2, false, edgeSplit);
//            } else if (!fixedCount && Math.PI - angle < 0.0){
//                refineArc(a, 0, true, 1, false, edgeSplit);
//            }
//            while ((!fixedCount && Point.distance(a.vrts.get(0), a.vrts.get(1)) > Surface.refineFac * maxLen) || (fixedCount && it < numOfSubdivisions)) {
//                List<Point> newVerts = new ArrayList<>();
//                for (int i = 0; i < a.vrts.size() - ((fullCircle) ? 0 : 1); ++i) {
//                    Point tmp = null;
//                    if (a.vrts.size() == 2){
//                        if (a.mid != null) {
//                            tmp = a.mid;
//                        } else {
//                            tmp = temp.assignTranslation(a.end1, v.changeVector(a.end2, a.end1).multiply(0.5f));
//                            v.changeVector(tmp, a.center).makeUnit().multiply(a.radius);
//                            if (n.assignNormalVectorOf(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0) {
//                                v.multiply(-1.0);
//                            }
//                            tmp = Point.translatePoint(a.center, v);
//                        }
//                    } else {
//                        if (i < a.vrts.size() - 1) {
//                            temp.assignTranslation(a.vrts.get(i), v.changeVector(a.vrts.get(i + 1), a.vrts.get(i)).multiply(0.5f));
//                        } else {
//                            temp.assignTranslation(a.vrts.get(i), v.changeVector(a.vrts.get(0), a.vrts.get(i)).multiply(0.5f));
//                        }
//                        v.changeVector(temp, a.center).makeUnit().multiply(a.radius);
//                        tmp = Point.translatePoint(a.center, v);
//                    }
//
//                    newVerts.add(a.vrts.get(i));
//                    newVerts.add(tmp);
//                    tmp._id = a.owner.nextVertexID++;
//                    a.owner.vertices.add(tmp);
//                    if (!fullCircle && i == a.vrts.size() - 2) {
//                        newVerts.add(a.vrts.get(i + 1));
//                    }
//                }
//                a.vrts = newVerts;
//                it++;
//            }
//            for (Point v : a.vrts){
//                //v.arcPoint = true;
//                v.arc = a;
//            }
//            if (a.baseSubdivision < 0){
//                a.baseSubdivision = getSubdivisionLevel(a);
//            }
//        } catch (Exception e){
//            e.printStackTrace();
//        }
//    }

    public static void refineOppositeArcs(Arc a1, Arc a2, double maxlen, boolean meshRefine, boolean convex){
        //List<Map<Integer, Map<Integer, Integer>>> edgeSplit = (convex) ? MeshGeneration.convexEdgeSplitMap : MeshGeneration.concaveEdgeSplitMap;
        Arc longer = (a1.radius - a2.radius > 0.0) ? a1 : a2;
        Arc shorter = (a1 == longer) ? a2 : a1;
        if (!meshRefine) {
            //int currentLevel = getSubdivisionLevel(shorter);
            //refineArc(shorter, maxlen, false, 0, false);
            //int numOfDivs = getSubdivisionLevel(shorter) - currentLevel;
            //refineArc(longer, 0, true, numOfDivs, false);
        } else {
            int currentLevel = getSubdivisionLevel(longer);
            int subdivisionDifference = currentLevel - getSubdivisionLevel(shorter);
            refineArc(longer, maxlen, false, 0, false);//, null);//edgeSplit.get(longer.owner.id));
            int numOfDivs = subdivisionDifference + getSubdivisionLevel(longer) - currentLevel;
            refineArc(shorter, 0, true, numOfDivs, false);//, null);//edgeSplit.get(shorter.owner.id));
            //for (Point v : shorter.vrts){
            //    v.arc = shorter;
            //}
            //for (Point v : longer.vrts){
            //    v.arc = longer;
            //}
        }
    }

    public static void buildEdges(Boundary b, boolean clear){
        if (clear) {
            b.vrts.clear();
            linkArcs(b.arcs);
            for (Arc a : b.arcs) {
                a.bOwner = b;
                a.owner = b.patch;
                a.valid = true;
                for (int i = 0; i < a.vrts.size() - 1; ++i) {
                    b.vrts.add(a.vrts.get(i));
                    a.vrts.get(i).arc = a;
                }
            }
        }
//        b.lines.clear();
//        for (int i = 0; i < b.vrts.size(); ++i){
//            Edge e = new Edge(i, (i == b.vrts.size() - 1) ? 0 : i + 1);
//            e.p1 = b.vrts.get(i);
//            e.p2 = b.vrts.get((i == b.vrts.size() - 1) ? 0 : i + 1);
//            b.lines.add(e);
//        }
//        for (int i = 0; i < b.lines.size(); ++i){
//            Edge first = b.lines.get(i);
//            Edge second = b.lines.get((i == b.lines.size() - 1) ? 0 : i + 1);
//            first.next = second;
//            second.prev = first;
//        }
    }

//    public static void buildEdges(Boundary b, boolean clear, double edgeLength){
//        if (clear) {
//            b.vrts.clear();
//            linkArcs(b.arcs);
//            for (Arc a : b.arcs) {
//                a.bOwner = b;
//                a.owner = b.patch;
//                a.valid = true;
//                int step = a.vrts.size() - 1;
//                while (getArcLength(a, a.vrts.get(0), a.vrts.get(step)) > 1.6 * edgeLength || step > 1){
//                    step /= 2;
//                }
//                for (int i = 0; i < a.vrts.size() - 1; i += step) {
//                    b.vrts.add(a.vrts.get(i));
//                }
//            }
//        }
//        b.lines.clear();
//        for (int i = 0; i < b.vrts.size(); ++i){
//            Edge e = new Edge(i, (i == b.vrts.size() - 1) ? 0 : i + 1);
//            e.p1 = b.vrts.get(i);
//            e.p2 = b.vrts.get((i == b.vrts.size() - 1) ? 0 : i + 1);
//            b.lines.add(e);
//        }
//        for (int i = 0; i < b.lines.size(); ++i){
//            Edge first = b.lines.get(i);
//            Edge second = b.lines.get((i == b.lines.size() - 1) ? 0 : i + 1);
//            first.next = second;
//            second.prev = first;
//        }
//    }

    public static double getAngleR(Arc a){
        double phi = Math.acos(a.toEnd1.dotProduct(a.toEnd2));
        if (Math.abs(a.toEnd1.dotProduct(a.toEnd2) + 1) < 0.001){
            return Math.PI;
        }
        if (n.assignNormalVectorOf(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){//Vector.getNormalVector(a.toEnd1, a.toEnd2).makeUnit().dotProduct(a.normal) < 0.0){
            phi = 2 * Math.PI - phi;
        }
        return phi;
    }

    private static Vector neighborToAtom = new Vector(0, 0, 0);
    private static Vector atomToNeighbor = new Vector(0, 0, 0);
    private static Vector atomToEnd = new Vector(0, 0, 0);
    private static Vector projection = new Vector(0, 0, 0);
    private static Vector u1 = new Vector(0, 0, 0);
    private static Vector u2 = new Vector(0, 0, 0);
    private static float[] _v = new float[3];
    private static Quaternion qRot = new Quaternion();
    //private static Vector n = new Vector(0, 0, 0);
    public static Arc[] makeNewArc(SphericalPatch sp, SphericalPatch neighbor, Point e1, Point e2, Point mid, Point midProbe, boolean circular){
        if (!sp.neighbours.contains(neighbor)){
            sp.neighbours.add(neighbor);
        }
        //Vector neighborToAtom = Point.subtractPoints(sp.sphere.center, neighbor.sphere.center);
        //Vector atomToNeighbor = Vector.scaleVector(neighborToAtom, -1);
        //Vector atomToEnd = Point.subtractPoints(e1, sp.sphere.center);
        //Vector projection = atomToEnd.projectionOnto(atomToNeighbor);
        neighborToAtom.changeVector(sp.sphere.center, neighbor.sphere.center);
        atomToNeighbor.changeVector(neighborToAtom).multiply(-1);
        atomToEnd.changeVector(e1, sp.sphere.center);
        //projection.changeVector(atomToNeighbor).multiply(atomToEnd.dotProduct(atomToNeighbor));
        projection = atomToEnd.projectionOnto(atomToNeighbor);
        Point arcCenter = Point.translatePoint(sp.sphere.center, projection);
        double loopRadius = Math.sqrt(sp.sphere.radius * sp.sphere.radius - projection.dotProduct(projection));
        neighborToAtom.makeUnit();
        Point end1 = e1;
        Point end2 = e2;
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
        //Vector e1ToMid = Point.subtractPoints(mid, e1);
        //Vector e1ToE2 = Point.subtractPoints(e2, e1);
        //Vector u1 = Point.subtractPoints(e1, arcCenter).makeUnit();
        //Vector u2 = Point.subtractPoints(mid, arcCenter).makeUnit();
        //Vector n = Vector.getNormalVector(u1, u2).makeUnit();
        u1.changeVector(e1, arcCenter).makeUnit();
        u2.changeVector(mid, arcCenter).makeUnit();
        n.assignNormalVectorOf(u1, u2).makeUnit();
        //e1ToMid.makeUnit();
        //e1ToE2.makeUnit();
        //Vector normal = Vector.getNormalVector(e1ToMid, e1ToE2);
        //normal.makeUnit();
        //Point triMid = Point.getMidPoint(e1, mid, e2);

        //Vector e1ToCenter = Point.subtractPoints(sp.sphere.center, arcCenter).makeUnit();
        if (n.dotProduct(neighborToAtom) < 0){
            end1 = e2;
            end2 = e1;
        }
        /*if (Point.subtractPoints(end1, mid).sqrtMagnitude() + Point.subtractPoints(end2, mid).sqrtMagnitude() < 0.001){
            System.err.println("possible short loop, atom1 id: " + sp.id + " atom2Id: " + neighbor.id);
        }*/
        Arc[] arc = new Arc[1];
        arc[0] = new Arc(arcCenter, loopRadius);
        Arc a = arc[0];
        //a.normal = Vector.scaleVector(neighborToAtom, 1).makeUnit();
        a.setNormal(neighborToAtom);
        a.normal.makeUnit();
        a.end1 = end1;
        a.end2 = end2;
        a.setEndPoints(a.end1, a.end2, false);
        //a.endEdge1 = new Edge(0, 1);
        //a.endEdge1.p1 = end1;
        //a.endEdge1.p2 = mid;
        //a.endEdge2 = new Edge(1, 2);
        //a.endEdge2.p1 = mid;
        //a.endEdge2.p2 = end2;
        //a.mid = mid;

        //a.endEdge1.next = a.endEdge2;
        //a.endEdge2.prev = a.endEdge1;

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
    private static List<Arc> queue = new ArrayList<>();
    private static List<Arc> newB = new ArrayList<>(10);
    public static void linkArcs(SphericalPatch sp) {
        try {
            //List<Arc> queue = new ArrayList<>(sp.arcs);
            queue.clear();
            queue.addAll(sp.arcs);

            boolean setValid = true;
            int loopEndIdx = 0;
            while (queue.size() > 0) {
                //ArrayList<Arc> newB = new ArrayList<>();
                newB.clear();
                Arc l = queue.get(loopEndIdx);
                newB.add(l);
                queue.remove(l);
                //Vector loopEnd = l.toEnd1;
                //Vector pivot = l.toEnd2;
                Point loopEnd = l.end1;
                Point pivot = l.end2;
                int i = 0;
                int iterator = 0;
                setValid = true;
                //while (Math.abs(loopEnd.dotProduct(pivot) - 1) >= 0.00001) {
                while (Point.distance(loopEnd, pivot) > 0.001) {//Point.subtractPoints(loopEnd, pivot).sqrtMagnitude() > 0.001) {
                    if (iterator > sp.arcs.size() + 10) {
                        loopEndIdx++;
                        if (loopEndIdx >= queue.size()) {
                            //System.err.println("Cycle detected for atom id:" + sp.id);
                            //System.out.println("Iterator: " + iterator);
                            //System.out.println("Queue size: " + queue.size());
                            //Main.convexPatches.remove(sp);
                            sp.valid = (sp.boundaries.size() > 0);
                            return;
                        } else {
                            //loopEndIdx++;
                            setValid = false;
                            break;
                        }
                        //return;
                    }
                    iterator++;
                    if (queue.size() == 0) {
                        //System.err.println("Atom " + sp.id);
                        setValid = false;
                        break;
                    }
                    Arc lop = queue.get(i);
                    //Vector vLop = lop.toEnd1;
                    Point pLop = lop.end1;
                    if (Point.distance(pLop, pivot) < 0.001) {//Point.subtractPoints(pLop, pivot).sqrtMagnitude() < 0.001) {
                        boolean betterCand = false;
                        Arc tmp = lop;
                        for (int j = i + 1; j < queue.size(); ++j) {
                            Arc tL = queue.get(j);
                            if (Point.distance(l.end2, tL.end1) < Point.distance(l.end2, lop.end1)) {//Point.subtractPoints(l.end2, tL.end1).sqrtMagnitude() < Point.subtractPoints(l.end2, lop.end1).sqrtMagnitude()) {
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
                        //lop.lines.get(0).p1 = l.end2;
                        lop.vrts.remove(0);
                        lop.vrts.add(0, l.end2);

                        //l.endEdge2.next = lop.endEdge1;
                        //lop.endEdge1.prev = l.endEdge2;
                        l = lop;
                        iterator = 0;
                    } else {
                        i++;
                    }
                    if (i >= queue.size()) {
                        i = 0;
                        //completeBoundary = false;
                        //System.err.println("atom id: " + this.id + " incomplete boundary");m10480
                        //return;
                    }
                }
                /*if (newB.size() < 2){
                    setValid = false;
                }*/
                if (setValid && newB.size() > 1) {
                    newB.get(0).end1 = l.end2;
                    //newB.get(0).lines.get(0).p1 = l.end2;
                    newB.get(0).vrts.remove(0);
                    newB.get(0).vrts.add(0, l.end2);
                    //newB.get(0).endEdge1.prev = l.endEdge2;
                    //l.endEdge2.next = newB.get(0).endEdge1;
                    Boundary b = new Boundary();
                    b.patch = sp;
                    //b.arcs = newB;
                    b.arcs.addAll(newB);
                    sp.boundaries.add(b);
                    ArcUtil.buildEdges(b, true);
                    for (int k = 0; k < b.vrts.size(); ++k){
                        Point v = b.vrts.get(k);
                        v._id = sp.nextVertexID++;
                        sp.vertices.add(v);
                    }
                    loopEndIdx = 0;
                    sp.valid = true;
                } else {
                    sp.valid = (sp.boundaries.size() > 0);
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
        //a.toEnd1 = Point.subtractPoints(a.end1, a.center).makeUnit();
        //a.toEnd2 = Point.subtractPoints(a.end2, a.center).makeUnit();
        Vector temp = a.toEnd1;
        a.toEnd1 = a.toEnd2;
        a.toEnd2 = temp;
        if (a.normal != null) {
            a.normal.multiply(-1.0);
        }
    }

    public static boolean checkIfNested(Boundary b1, Boundary b2){
        return (checkForOwnership(b1, b2) && checkForOwnership(b2, b1));
    }

    private static boolean checkForOwnership(Boundary b1, Boundary b2){
        try {
            Point p1 = b1.arcs.get(0).end1;
            //Vector v1 = null;
            //Vector v2 = null;
            if (b1.arcs.size() < 2){
                return false;
            }
            for (int i = 1; i < b1.arcs.size(); ++i) {
                Arc l = b1.arcs.get(i);
                //v1 = Point.subtractPoints(l.end2, p1).makeUnit();
                //v2 = Point.subtractPoints(l.end1, p1).makeUnit();
                u1.changeVector(l.end2, p1).makeUnit();
                u2.changeVector(l.end1, p1).makeUnit();
                if (Math.abs(u1.dotProduct(u2) - 1.0) > 0.01) {
                    break;
                }
            }
            //Vector n = Vector.getNormalVector(v1, v2).makeUnit().multiply(-1.0);
            n.assignNormalVectorOf(u1, u2).makeUnit().multiply(-1.0);
            double maxY = 42000.0;
            for (int i = 0; i < b1.arcs.size(); ++i) {
                Arc l = b1.arcs.get(i);
                //Vector u = Point.subtractPoints(l.end2, b1.patch.sphere.center);
                u.changeVector(l.end2, b1.patch.sphere.center);
                double dot = u.dotProduct(n);
                if (dot < maxY) {
                    maxY = dot;
                }
            }
            boolean isInside = true;
            for (int i = 0; i < b2.arcs.size(); ++i) {
                Arc l = b2.arcs.get(i);
                //Vector u = Point.subtractPoints(l.end2, b2.patch.sphere.center);
                u.changeVector(l.end2, b2.patch.sphere.center);
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

    private static float[] _vData = new float[3];
    private static float[] _vData2 = new float[3];
    public static List<Point> generateCircArc(Point start, Point end, Point center, double radius, int n, boolean overPI, List<Point> vrtsList){
        //Vector tostart = Point.subtractPoints(start, center).makeUnit();
        //Vector toend = Point.subtractPoints(end, center).makeUnit();
        //Vector axis = Vector.getNormalVector(toend, tostart).makeUnit();
        /*if (tostart.dotProduct(toend) < 0.0){
            axis.multiply(-1);
        }*/
        v1.changeVector(start, center).makeUnit(); //tostart
        v2.changeVector(end, center).makeUnit(); //toend
        v.assignNormalVectorOf(v2, v1).makeUnit(); //axis of rotation
        //double angle = Math.acos(toend.dotProduct(tostart));
        double angle = Math.acos(v2.dotProduct(v1));
        if (overPI){
            angle = 2 * Math.PI - angle;
            v.multiply(-1);
        }
        _vData[0] = (float)v.getX();
        _vData[1] = (float)v.getY();
        _vData[2] = (float)v.getZ();

        _vData2[0] = (float)v1.getX();
        _vData2[1] = (float)v1.getY();
        _vData2[2] = (float)v1.getZ();
        /*if (angle - Math.PI >= 0.0){
            System.out.println(" ");
        }*/
        angle = angle / n;
        List<Point> vrts = (vrtsList == null) ? new ArrayList<>() : vrtsList;
        vrts.add(start);
        if (n > 1) {
            for (int i = 1; i < n; ++i) {
                //Quaternion qRot = new Quaternion(0, 0, 0, 0);

                qRot.setFromAngleNormalAxis((float) (-i * angle), _vData);// v.getFloatData());
                //float[] v = new float[3];
                //v = qRot.rotateVector(v, 0, v1.getFloatData(), 0);
                qRot.rotateVector(_v, 0, _vData2, 0);//v1.getFloatData(), 0);
                //Vector u = new Vector(v);
                //u.makeUnit().multiply(radius);
                u.changeVector(_v[0], _v[1], _v[2]).makeUnit().multiply(radius);
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
        //a.mid = newMid;
    }

    private static final double PLANE_EPS = 0.001;
    private static Plane plane = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));
    private static Vector _fcaVector = new Vector(0, 0, 0);

    public static Arc findContainingArc(Point p, Plane circle, SphericalPatch sp, Arc exclude){
        for (int i = 0; i < sp.boundaries.size(); ++i){
            Boundary b = sp.boundaries.get(i);
            for (int j = 0; j < b.arcs.size(); ++j){
                Arc a = b.arcs.get(j);
                if (a == exclude){
                    continue;
                }
                //Plane plane = new Plane(a.center, a.normal);
                plane.redefine(a.center, a.normal);
                if (Math.abs(plane.checkPointLocation(p)) > PLANE_EPS){
                    continue;
                }
                //Vector vector = Point.subtractPoints(p, a.center).makeUnit();
                _fcaVector.changeVector(p, a.center).makeUnit();
                if (Math.abs(_fcaVector.dotProduct(a.toEnd1) - 1.0) < 0.001){
                    /*Point _p = PatchUtil.genP(a, p);
                    Vector _v = Point.subtractPoints(_p, circle.p).multiply(10.f);
                    Point _p2 = Point.translatePoint(circle.p, _v);
                    if (circle.checkPointLocation(_p2) < 0.0){
                        return a.prev;
                    }*/
                    double aNextSign = PatchUtil.nextSign(p, a, circle);
                    double aprevNextSign = PatchUtil.nextSign(p, a.prev, circle);
                    if (aNextSign * aprevNextSign < 0.0){
                        return null;
                    }
                    if (aNextSign > 0.0){
                        return a;
                    }
                    return a.prev;
                } else if (Math.abs(_fcaVector.dotProduct(a.toEnd2) - 1.0) < 0.001){
                    /*Point _p = PatchUtil.genP(a, p);
                    Vector _v = Point.subtractPoints(_p, circle.p).multiply(10.f);
                    Point _p2 = Point.translatePoint(circle.p, _v);
                    if (circle.checkPointLocation(_p2) < 0.0){
                        return a;
                    }
                    return a.next;*/
                    double aNextSign = PatchUtil.nextSign(p, a, circle);
                    double anextNextSign = PatchUtil.nextSign(p, a.next, circle);
                    if (aNextSign * anextNextSign < 0.0){
                        return null;
                    }
                    if (aNextSign < 0.0){
                        return a;
                    }
                    return a.next;
                } else {
                    if (a.isInside(p)){
                        return a;
                    }
                }
            }
        }
        return null;
    }

    public static Arc _findContainingArc(Point p, Plane circle, SphericalPatch sp, Arc exclude){
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (a == exclude){
                    continue;
                }
                //Plane plane = new Plane(a.center, a.normal);
                plane.redefine(a.center, a.normal);
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
    private static Vector toStart = new Vector(0, 0, 0);
    private static Vector toP = new Vector(0, 0, 0);
    private static Vector v1 = new Vector(0, 0, 0);
    private static Vector v2 = new Vector(0, 0, 0);
    public static Point findClosestPointOnCircle(List<Point> points, Point start, boolean includeStart, Point center, Vector normal, boolean next){
        double angle = 2 * Math.PI;
        Point closest = null;
        //Vector toStart = Point.subtractPoints(start, center).makeUnit();
        toStart.changeVector(start, center).makeUnit();
        for (int i = 0; i < points.size(); ++i){
            Point p = points.get(i);
            if (includeStart && Point.distance(start, p) < 0.001){
                return p;
            }
            //Vector toP = Point.subtractPoints(p, center).makeUnit();
            toP.changeVector(p, center).makeUnit();
            double alpha = (v1.assignNormalVectorOf(toStart, toP).multiply(next ? 1 : -1).dotProduct(normal) > 0.0) ? Math.acos(toStart.dotProduct(toP)) : (2 * Math.PI - Math.acos(toStart.dotProduct(toP)));
            if (Math.abs(toStart.dotProduct(toP) - 1.0) < 0.001 && includeStart){
                angle = 0.0;
                closest = p;
            }
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
            curr.vrts.remove(0);
            curr.vrts.add(0, curr.end1);
            curr.end2 = next.end1;
            curr.vrts.remove(curr.vrts.size() - 1);
            curr.vrts.add(next.end1);
        }
    }

    public static int getOrder(Arc a, Point p, Point q){
        //Vector pV = Point.subtractPoints(p, a.center).makeUnit();
        //Vector qV = Point.subtractPoints(q, a.center).makeUnit();
        v1.changeVector(p, a.center).makeUnit();//pV
        v2.changeVector(q, a.center).makeUnit();//qV
        double phi1 = Math.acos(v1.dotProduct(a.toEnd1));
        double phi2 = Math.acos(v2.dotProduct(a.toEnd1));
        phi1 = (n.assignNormalVectorOf(a.toEnd1, v1).makeUnit().dotProduct(a.normal) > 0.0) ? phi1 : 2 * Math.PI - phi1;//Vector.getNormalVector(a.toEnd1, v1).makeUnit().dotProduct(a.normal) > 0.0) ? phi1 : 2 * Math.PI - phi1;
        phi2 = (n.assignNormalVectorOf(a.toEnd1, v2).makeUnit().dotProduct(a.normal) > 0.0) ? phi2 : 2 * Math.PI - phi2;//Vector.getNormalVector(a.toEnd1, v2).makeUnit().dotProduct(a.normal) > 0.0) ? phi2 : 2 * Math.PI - phi2;
        return (phi1 - phi2 < 0.0) ? -1 : 1;
    }

    public static Arc dbgCloneArc(Arc a){
        Arc na = new Arc(a.center, a.radius);
        na.setEndPoints(a.end1, a.end2, false);
        //na.mid = a.mid;
        na.setNormal(a.normal);
        na.vrts.addAll(a.vrts);
        na.baseSubdivision = a.baseSubdivision;
        return na;
    }

    /*
        copies the arc a into new one, cloning its vertices instead of just sharing them with the original arc
     */
    public static Arc cloneArc(Arc a){
        Arc newA = new Arc(a.center, a.radius);
        //a.vrts.forEach(v -> newA.vrts.add(new Point(v)));
        for (int i = 0; i < a.vrts.size(); ++i){
            newA.vrts.add(new Point(a.vrts.get(i)));
        }
        newA.setNormal(a.normal);
        newA.setEndPoints(newA.vrts.get(0), newA.vrts.get(newA.vrts.size() - 1), false);
        newA.baseSubdivision = a.baseSubdivision;
        return newA;
    }

    private static Vector _perpendicular = new Vector(0, 0, 0);
    public static Boundary generateCircularBoundary(Plane circle, double radius){
        circle.v.getPerpendicularVector(_perpendicular).makeUnit().multiply(radius);
        //Vector perp2 = Vector.getNormalVector(circle.v, perp).makeUnit().multiply(radius);
        n.assignNormalVectorOf(circle.v, _perpendicular).makeUnit().multiply(radius); //perp2
        Arc a1 = new Arc(circle.p, radius);
        Point p1 = Point.translatePoint(circle.p, _perpendicular);
        Point mid1 = Point.translatePoint(circle.p, n);
        n.multiply(-1);
        Point mid2 = Point.translatePoint(circle.p, n);
        _perpendicular.multiply(-1);
        Point p2 = Point.translatePoint(circle.p, _perpendicular);
        a1.setEndPoints(p1, p2, false);
        a1.setNormal(circle.v);
        //a1.mid = mid1;
        a1.vrts.add(p1);
        a1.vrts.add(mid1);
        a1.vrts.add(p2);
        //ArcUtil.refineArc(a1, 0, true, 1, false);
        ArcUtil.refineArc(a1, Surface.maxEdgeLen, false, 0, false);
        Arc a2 = new Arc(circle.p, radius);
        a2.setEndPoints(p2, p1, false);
        a2.setNormal(circle.v);
        //a2.mid = mid2;
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
    private static float[] _dirVec = new float[3];
    public static void redefineBoundary(Boundary b, Plane circle, double radius, List<Point> vrts, double angleD){
        circle.v.getPerpendicularVector(_perpendicular).makeUnit().multiply(radius);
        n.assignNormalVectorOf(circle.v, _perpendicular).makeUnit().multiply(radius);
        int vrtsIndex = 0;
        Point start = vrts.get(vrtsIndex++);
        Point end = vrts.get(vrtsIndex++);
        start.assignTranslation(circle.p, n);
        end.assignTranslation(circle.p, n.multiply(-1.0));
        n.multiply(-1.0);
        int count = (int)(180 / angleD);
        qRot.setFromAngleNormalAxis((float)Math.toRadians(angleD), circle.v.getFloatData());
        Arc a1 = b.arcs.get(0);
        Arc a2 = b.arcs.get(1);
        a1.vrts.clear();
        a2.vrts.clear();
        a1.vrts.add(start);
        a2.vrts.add(end);
        _dirVec[0] = (float)n.getX();
        _dirVec[1] = (float)n.getY();
        _dirVec[2] = (float)n.getZ();
        for (int i = 1; i < count; ++i){
            if (vrtsIndex >= vrts.size()){
                vrts.add(new Point(0, 0, 0));
                vrts.add(new Point(0, 0, 0));
            }
            qRot.rotateVector(_dirVec, 0, _dirVec, 0);
            v1.changeVector(_dirVec[0], _dirVec[1], _dirVec[2]).makeUnit().multiply(radius);
            Point p1 = vrts.get(vrtsIndex++);
            Point p2 = vrts.get(vrtsIndex++);
            p1.assignTranslation(circle.p, v1);
            p2.assignTranslation(circle.p, v1.multiply(-1.0));
            a1.vrts.add(p1);
            a2.vrts.add(p2);
        }
        a1.vrts.add(end);
        a2.vrts.add(start);
        a1.center.change(circle.p);
        a2.center.change(circle.p);
        a1.radius = radius;
        a2.radius = radius;
        a1.setEndPoints(start, end, false);
        a2.setEndPoints(end, start, false);
        a1.setNormal(circle.v);
        a2.setNormal(circle.v);
    }

    public static byte getSubdivisionLevel(Arc a){
        return (byte)(Math.log10(a.vrts.size() - 1) / Math.log10(2));
    }

    /*public static void markShared(Arc a){
        a.vrts.stream().forEach(v -> v.isShared = true);
    }*/

    //public static void generateEdgeSplits(Arc a, SphericalPatch sp){
    //    Map<Integer, Map<Integer, Integer>> edgeSplit = (sp.convexPatch) ? MeshGeneration.convexEdgeSplitMap.get(sp.id) : MeshGeneration.concaveEdgeSplitMap.get(sp.id);
    //    generateEdgeSplits(0, a.vrts.size() - 1, a, edgeSplit);
    //}

    //private static void generateEdgeSplits(int start, int count, Arc a, Map<Integer, Map<Integer, Integer>> splits){
    //    if (count == 1){
    //        return;
    //    }
    //    Point v1 = a.vrts.get(start);
    //    Point v2 = a.vrts.get(start + count);
    //    Point v3 = a.vrts.get(start + count / 2);
    //    int sID = (v1._id > v2._id) ? v2._id : v1._id;
    //    int bID = (v1._id > v2._id) ? v1._id : v2._id;
    //    if (!splits.containsKey(sID)){
    //        splits.put(sID, new TreeMap<>());
    //    }
    //    Map<Integer, Integer> map = splits.get(sID);
    //    map.put(bID, v3._id);
    //    generateEdgeSplits(start, count / 2, a, splits);
    //    generateEdgeSplits(start + count / 2, count / 2, a, splits);
    //}

//    public static Arc getCommonArc(Point p1, Point p2){
//        if (!p1.arcPoint || !p2.arcPoint){
//            return null;
//        }
//        if ((p1 == p1.arc.end1 || p1 == p1.arc.end2) && (p2 == p2.arc.end1 || p2 == p2.arc.end2)){
//            Boundary b = p1.arc.bOwner;
//            try {
//                for (Arc a : b.arcs) {
//                    if ((p1 == a.end1 && p2 == a.end2) || (p1 == a.end2 && p2 == a.end1)) {
//                        //System.out.println("found the arc of " + a.vrts.size() + " vrts");
//                        return a;
//                    }
//                }
//            } catch (Exception e){
//                e.printStackTrace();
//            }
//            return null;
//        }
//        if (p1 == p1.arc.end1 || p1 == p1.arc.end2){
//            if (p1 == p2.arc.end1 || p1 == p2.arc.end2) {
//                return p2.arc;
//            }
//            return null;
//        }
//        if (p2 == p2.arc.end1 || p2 == p2.arc.end2){
//            if (p2 == p1.arc.end1 || p2 == p1.arc.end2) {
//                return p1.arc;
//            }
//            return null;
//        }
//        if (p1.arc == p2.arc){
//            return p1.arc;
//        } else {
//            return null;
//        }
//    }

//    public static void refineArcsOnConcavePatches(){
//        for (SphericalPatch sp : Surface.triangles) {
//            sp.trimmed = true;
//            sp.vertices.clear();
//            sp.nextVertexID = 0;
//            for (Boundary b : sp.boundaries) {
//                ArcUtil.buildEdges(b, true);
//                for (Point v : b.vrts) {
//                    v._id = sp.nextVertexID++;
//                    sp.vertices.add(v);
//                }
//                for (Arc a : b.arcs) {
//                    a.owner = sp;
//                    if (false && a.opposite != null && a.refined == null) {
//                        a.refined = ArcUtil.dbgCloneArc(a);
//                        a.refined.owner = a.owner;
//                        if (a.owner.intersectingPatches.contains(a.opposite.owner.id) && a.cuspTriangle == null) {
//                            ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0, false);//, null);//MeshGeneration.concaveEdgeSplitMap.get(a.owner.id));
//                            a.opposite.refined = ArcUtil.cloneArc(a.refined);
//                            ArcUtil.reverseArc(a.opposite.refined, true);
//                            int c = 432;
//                        } else {
//                            //int curr = getSubdivisionLevel(a.opposite);
//                            a.opposite.refined = dbgCloneArc(a.opposite);
//                            a.opposite.refined.owner = a.opposite.owner;
//                            refineOppositeArcs(a.refined, a.opposite.refined, SesConfig.edgeLimit, true, false);
//                            //refineArc(a.opposite.refined, SesConfig.edgeLimit, true, getSubdivisionLevel(a.refined) - curr, false, MeshRefinement.concaveEdgeSplitMap.get(a.opposite.owner.id));
//                            //System.out.println("refining cusp triangle arcs");
//                        }
//                        a.opposite.refined.owner = a.opposite.owner;
//                        if (a.refined.owner == null || a.opposite.refined.owner == null) {
//                            int c = 4;
//                        }
//                        if (sp.trimmed) {
//                            a.vrts.clear();
//                            a.vrts.addAll(a.refined.vrts);
//                        }
//                        if (a.opposite.owner.trimmed) {
//                            a.opposite.vrts.clear();
//                            a.opposite.vrts.addAll(a.opposite.refined.vrts);
//                        }
//                    } else if (a.refined == null) {
//                        a.refined = ArcUtil.dbgCloneArc(a);
//                        a.refined.owner = a.owner;
//                        if (a.refined.owner == null) {
//                            int c = 4;
//                        }
//                        try {
//                            ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0, false);//, null);//MeshGeneration.concaveEdgeSplitMap.get(a.owner.id));
//                        } catch (Exception e) {
//                            e.printStackTrace();
//                        }
//                        if (sp.trimmed) {
//                            a.vrts.clear();
//                            a.vrts.addAll(a.refined.vrts);
//                        }
//                    }
//                    if (a.refined != null) {
//                        if (sp.trimmed) {
//                            a.vrts.clear();
//                            a.vrts.addAll(a.refined.vrts);
//                        }
//                        for (Point v : a.vrts) {
//                            if (v._id < 0) {
//                                v._id = sp.nextVertexID++;
//                                sp.vertices.add(v);
//                            }
//                        }
//                        //generateEdgeSplits(a.refined, a.owner);
//                    }
//                }
//                ArcUtil.buildEdges(b, true);
//            /*for (Point v : b.vrts){
//                v._id = sp.nextVertexID++;
//                sp.vertices.add(v);
//            }*/
//                for (Arc a : b.arcs) {
//                    for (Point p : a.vrts) {
//                        p.arc = a;
//                    }
//                }
//            }
//        }
//    }

    private static double getArcLength(Arc a, Point p1, Point p2){
        //Vector v1 = Point.subtractPoints(p1, a.center).makeUnit();
        //Vector v2 = Point.subtractPoints(p2, a.center).makeUnit();
        //Vector n = Vector.getNormalVector(v1, v2).makeUnit();
        u.changeVector(p1, a.center).makeUnit();
        v.changeVector(p2, a.center).makeUnit();
        n.assignNormalVectorOf(u, v).makeUnit();
        if (n.dotProduct(a.normal) < 0.0){
            return a.radius * (2 * Math.PI - Math.acos(u.dotProduct(v)));
        }
        return a.radius * Math.acos(u.dotProduct(v));
    }

    public static void resetArcs(SphericalPatch sp){
        sp.vertices.clear();
        sp.nextVertexID = 0;
        //sp.arcPointCount = 0;
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (a.bOwner == null){
                    int sd = 3;
                }
                resetArc(a);
                if (a.bOwner == null){
                    int c = 4;
                }
            }
            ArcUtil.buildEdges(b, true);
            for (Point p : b.vrts){
                p._id = sp.nextVertexID++;
                sp.vertices.add(p);
            }
        }
    }

    private static void resetArc(Arc a){
        if (a.baseSubdivision < 0){
            int c = 32;
        }
        byte currLevel = getSubdivisionLevel(a);
        if (a.baseSubdivision > currLevel){
            int s = 2;
        }
        if (a.baseSubdivision == currLevel){
            //a.refined.vrts.clear();
            //a.refined = null;
            return;
        }
        List<Point> tmp = new ArrayList<>(a.vrts);
        a.vrts.clear();
        int step = (int)Math.pow(2, currLevel - a.baseSubdivision);
        for (int i = 0; i < tmp.size(); i += step){
            a.vrts.add(tmp.get(i));
        }
        //a.refined.vrts.clear();
        //a.refined = null;
        if (getSubdivisionLevel(a) != a.baseSubdivision){
            int c = 43;
        }
    }

//    public static void refineArcsOnConvexPatches(){
//        for (SphericalPatch sp : Surface.convexPatches){
//            try {
//                if (sp.boundaries.size() == 0) {
//                    ArcUtil.linkArcs(sp);
//                }
//                if (sp.id == 1566 || sp.id == 885) {
//                    int ca = 42;
//                }
//                for (Boundary b : sp.boundaries) {
//                    if (b.arcs.size() < 2) {
//                        int c = 4;
//                    }
//                    for (Arc a : b.arcs) {
//                        //for (Point v : a.vrts) {
//                        //    v.arcPoint = true;
//                        //}
//                        if (a.refined != null) {
//                            continue;
//                        }
//                        Arc op = a.opposite;
//                        if (sp.id == 872 && op.owner.id == 885) {
//                            int fgd = 3;
//                        }
//                        if (op.owner.boundaries.size() == 0) {
//                            ArcUtil.linkArcs(op.owner);
//                        }
//                        a.refined = ArcUtil.dbgCloneArc(a);
//                        a.refined.owner = sp;
//                        op.refined = ArcUtil.dbgCloneArc(op);
//                        op.refined.owner = op.owner;
//                        ArcUtil.refineOppositeArcs(a.refined, op.refined, SesConfig.edgeLimit, true, true);
//                        //ArcUtil.generateEdgeSplits(a.refined, sp);
//                        //ArcUtil.generateEdgeSplits(op.refined, op.owner);
//                        a.vrts.clear();
//                        a.vrts.addAll(a.refined.vrts);
//                        op.vrts.clear();
//                        op.vrts.addAll(op.refined.vrts);
//                        for (Point v : a.vrts) {
//                            //v.arcPoint = true;
//                            v.arc = a;
//                        }
//                        for (Point v : op.vrts) {
//                            //v.arcPoint = true;
//                            v.arc = op;
//                        }
//                    }
//                    ArcUtil.buildEdges(b, true);
//                }
//                //refine arcs - when refining an arc, its opposite arc will be refined as well as to have the same number of vertices on both of them
//                Surface.atomsProcessed.set(sp.id + 1);
//            } catch (Exception e){
//                e.printStackTrace();
//            }
//        }
//    }

    public static void constructConvexBoundaries(){
        for (SphericalPatch sp : Surface.convexPatches){
            linkArcs(sp);
        }
    }

    public static void refineArcsOnSphericalPatches(){
        try {
            for (SphericalPatch sp : Surface.convexPatches) {
                for (int i = 0; i < sp.boundaries.size(); ++i) {
                    Boundary b = sp.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j) {
                        Arc a = b.arcs.get(j);
                        Arc opposite = a.opposite;
                        if (opposite.owner.id < sp.id) {
                            continue;
                        }
                        double edgeLimit = (b.arcs.size() > 2) ? SesConfig.edgeLimit : 0.75 * SesConfig.edgeLimit;
                        refineOppositeArcs(a, opposite, edgeLimit, true, true);
                    }
                    buildEdges(b, true);
                }
            }
            for (SphericalPatch sp : Surface.triangles) {
                for (int i = 0; i < sp.boundaries.size(); ++i) {
                    Boundary b = sp.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j) {
                        Arc a = b.arcs.get(j);
                        refineArc(a, SesConfig.edgeLimit, false, 0, false);
                    }
                    buildEdges(b, true);
                }
            }
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static boolean inco(SphericalPatch sp){
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (a.baseSubdivision > getSubdivisionLevel(a)){
                    return true;
                }
            }
        }
        return false;
    }

    public static void nestConvexPatchBoundaries(){
        for (SphericalPatch sp : Surface.convexPatches) {
            for (int i = 0; i < sp.boundaries.size(); ++i) {
                Boundary b = sp.boundaries.get(i);
                for (int j = 0; j < sp.boundaries.size(); ++j) {
                    Boundary c = sp.boundaries.get(j);
                    if (c == b) {
                        continue;
                    }
                    if (ArcUtil.checkIfNested(b, c)) {
                        b.nestedBoundaries.add(c);
                    }
                }
            }
        }
    }

    public static void indexPoints(SphericalPatch sp){
        sp.vertices.clear();
        sp.nextVertexID = 0;
        for (int i = 0; i < sp.boundaries.size(); ++i){
            Boundary b = sp.boundaries.get(i);
            for (int j = 0; j < b.vrts.size(); ++j){
                Point p = b.vrts.get(j);
                p._id = sp.nextVertexID++;
                sp.vertices.add(p);
            }
        }
    }

}
