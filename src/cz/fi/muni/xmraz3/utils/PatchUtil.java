package cz.fi.muni.xmraz3.utils;

import com.jogamp.opengl.math.Quaternion;
import cz.fi.muni.xmraz3.*;
import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Sphere;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.mesh.*;
import smile.neighbor.Neighbor;

import java.util.*;
import java.util.function.Predicate;

public class PatchUtil {
    private static Vector v1 = new Vector(0, 0, 0);
    private static Vector v2 = new Vector(0, 0, 0);
    private static Vector v3 = new Vector(0, 0, 0);
    private static Plane p1 = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));
    private static Plane p2 = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));
    public static double getProbeAxisDistance(Point probe, Point a1, Point a2){
        //Vector axis = Point.subtractPoints(a2, a1).makeUnit();
        //Vector toProbe = Point.subtractPoints(probe, a1);
        v1.changeVector(a2, a1).makeUnit(); //axis vector
        v2.changeVector(probe, a1); //vector to probe
        //axis.multiply(toProbe.dotProduct(axis));
        v1.multiply(v2.dotProduct(v1));
        //return Vector.addVectors(toProbe, axis.multiply(-1)).sqrtMagnitude();
        return v3.assignAddition(v2, v1.multiply(-1)).sqrtMagnitude();
    }
    private static Vector circleN = new Vector(0, 0, 0);
    private static Point point = new Point(0, 0, 0);
    private static List<Point> middlevrts = new ArrayList<>(17);
    public static void torProcessSelfIntersection(ToroidalPatch tp){
        try {
            if (!tp.circular) {
                if (tp.concavePatchArcs.size() < 2) {
                    System.out.println("damaged rolling patch");
                    tp.valid = false;
                    return;
                }
            }
            if (!tp.concavePatchArcs.get(0).valid || !tp.concavePatchArcs.get(1).valid){
                tp.valid = false;
            }
            if (!tp.valid){
                Surface.rectangles.remove(tp);
                //System.out.println(" errorrrr");
                return;
            }
            //System.err.println(rp.cxpl1.atom.id + " " + rp.cxpl2.atom.id);
            Arc leftL = tp.concavePatchArcs.get(0);
            Arc rightL = tp.concavePatchArcs.get(1);
            //Arc_ bottom = (leftL.end2 == tp.convexPatchArcs.get(0).end2) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
            Arc bottom = (Point.distance(leftL.end2, tp.convexPatchArcs.get(0).end2) < 0.001) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
            Arc top = (tp.convexPatchArcs.get(0) == bottom) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);
            if (leftL.owner.id == 30419 || rightL.owner.id == 30419){
                int g = 32;
            }
            if (leftL.owner.intersectingPatches.contains(rightL.owner.id)){
                //Arc cpl1 = leftL.owner.boundaries.get(0).arcs.stream().filter(a -> a.intersecting).findFirst().get();
                //Arc cpl2 = rightL.owner.boundaries.get(0).arcs.stream().filter(a -> a.intersecting).findFirst().get();
                Arc cpl1 = null;
                Arc cpl2 = null;
                for (int i = 0; i < leftL.owner.boundaries.get(0).arcs.size(); ++i){
                    Arc a = leftL.owner.boundaries.get(0).arcs.get(i);
                    if (a.intersecting){
                        cpl1 = a;
                        break;
                    }
                }
                for (int i = 0; i < rightL.owner.boundaries.get(0).arcs.size(); ++i){
                    Arc a = rightL.owner.boundaries.get(0).arcs.get(i);
                    if (a.intersecting){
                        cpl2 = a;
                        break;
                    }
                }
                //Main.processSelfIntersectingConcavePatch(cpl1);
                //Main.processSelfIntersectingConcavePatch(cpl2);
                processIntersectingArcsOnPatch(cpl2);
                processIntersectingArcsOnPatch(cpl1);

                SphericalPatch left = cpl1.owner;
                SphericalPatch right = cpl2.owner;

                CuspTriangle tr1 = new CuspTriangle();
                CuspTriangle tr2 = new CuspTriangle();

                tr1.base = bottom;
                tr2.base = top;

                Arc target = null;
                for (int i = 0; i < left.boundaries.size(); ++i){
                    Boundary b = left.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j){
                        Arc a = b.arcs.get(j);
                        if (Point.distance(tr1.base.end2, a.end2) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr1.left = target;
                if (tr1.left == null){
                    System.out.println(" ");
                }
                tr1.cuspPoint = tr1.left.end1;
                target = null;
                for (int i = 0; i < right.boundaries.size(); ++i){
                    Boundary b = right.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j){
                        Arc a = b.arcs.get(j);
                        if (Point.distance(tr1.base.end1, a.end1) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr1.right = target;
                target = null;
                for (int i = 0; i < right.boundaries.size(); ++i){
                    Boundary b = right.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j){
                        Arc a = b.arcs.get(j);
                        if (Point.distance(tr2.base.end2, a.end2) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr2.left = target;
                tr2.cuspPoint = tr2.left.end1;
                target = null;
                for (int i = 0; i < left.boundaries.size(); ++i){
                    Boundary b = left.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j){
                        Arc a = b.arcs.get(j);
                        if (Point.distance(tr2.base.end1, a.end1) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr2.right = target;
                tp.tr1 = tr1;
                tp.tr2 = tr2;
                //equalizeVertexCount(tp.tr1, tp.tr2);
                return;
            }
            if (tp.id == 8871){
                int f = 3;
            }
            Point circle = new Point(0, 0, 0);
            double radius = computeIntersectionCircle(leftL.owner.sphere.center, rightL.owner.sphere.center, circle, SesConfig.probeRadius);


            //Boundary[] bs = ConcavePatchUtil.generateCircularBoundary(leftL.owner, rightL.owner, circle, radius);
            //Vector circleN = Point.subtractPoints(leftL.owner.sphere.center, circle).makeUnit();
            circleN.changeVector(leftL.owner.sphere.center, circle).makeUnit();
            //Plane cPlane = new Plane(circle, circleN);
            //Point[] ccc = Util.getCusps(cPlane, circle, radius, leftL.lines);
            //Point[] cusps = Util.getCuspPoints(leftL.vrts, leftL.center, rightL.center, SesConfig.probeRadius);
            Point[] cusps = new Point[2];
            cusps[0] = computeCusp(tp.probe1, tp.convexPatchArcs.get(0).owner.sphere, tp.convexPatchArcs.get(1).owner.sphere);
            cusps[1] = computeCusp(tp.probe1, tp.convexPatchArcs.get(1).owner.sphere, tp.convexPatchArcs.get(0).owner.sphere);

            if (cusps[0] == null || cusps[1] == null){
                return;
            }
            if (Point.distance(leftL.end1, cusps[0]) - Point.distance(leftL.end1, cusps[1]) > 0.0) {
                Point tmp = cusps[1];
                cusps[1] = cusps[0];
                cusps[0] = tmp;
            }

            //Vector cusp1To0 = Point.subtractPoints(cusps[0], cusps[1]).makeUnit();
            //Vector perpendi = Vector.getNormalVector(circleN, cusp1To0).makeUnit();
            //Plane p = new Plane(cusps[1], perpendi);
            //Plane ro = new Plane(circle, perpendi);

            /*if (Math.abs(Math.abs(Point.subtractPoints(tp.convexPatchArcs.get(0).owner.sphere.center, tp.convexPatchArcs.get(1).owner.sphere.center).makeUnit().dotProduct(cusp1To0)) - 1) > 0.001){
                System.out.println("not parallel vectors bro");
            }*/

            //Point[] cusps = Util.getCuspPoints(leftL.vrts, leftL.center, rightL.center, SesConfig.probeRadius);

            Arc leftStart = new Arc(leftL.center, leftL.radius);
            leftStart.owner = leftL.owner;
            //leftStart.end1 = leftL.end1;
            //leftStart.end2 = cusps[0];
            leftStart.setEndPoints(leftL.end1, cusps[0], true);
            leftStart.prev = leftL.prev;
            leftStart.prev.next = leftStart;
            leftStart.vrts.add(leftStart.end1);
            leftStart.vrts.add(cusps[0]);
            //leftStart.refineLoop(Main.maxEdgeLen, 0.0, true, 1, false);
            //leftStart.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
            //ArcUtil.refineArc(leftStart, Surface.maxEdgeLen, true,1, false);
            ArcUtil.refineArc(leftStart, Surface.maxEdgeLen, false, 0, false);
            //leftStart.endEdge1 = new Edge(0, 1);
            //leftStart.endEdge1.p1 = leftStart.end1;
            //leftStart.endEdge1.p2 = leftStart.vrts.get(1);
            //leftStart.endEdge2 = new Edge(leftStart.vrts.size() - 2, leftStart.vrts.size() - 1);
            //leftStart.endEdge2.p1 = leftStart.vrts.get(leftStart.vrts.size() - 2);
            //leftStart.endEdge2.p2 = leftStart.end2;
            //leftStart.buildEdges();



            Arc leftEnd = new Arc(leftL.center, leftL.radius);
            leftEnd.owner = leftL.owner;
            //leftEnd.end1 = cusps[1];
            //leftEnd.end2 = leftL.end2;
            leftEnd.setEndPoints(cusps[1], leftL.end2, true);
            leftEnd.next = leftL.next;
            leftEnd.next.prev = leftEnd;
            leftEnd.vrts.add(cusps[1]);
            leftEnd.vrts.add(leftEnd.end2);
            //leftEnd.refineLoop(Main.maxEdgeLen, 0.0, true, 1, false);
            //leftEnd.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
            //ArcUtil.refineArc(leftEnd, Surface.maxEdgeLen, true, 1, false);
            ArcUtil.refineArc(leftEnd, Surface.maxEdgeLen, false, 0, false);
            //leftEnd.endEdge1 = new Edge(0, 1);
            //leftEnd.endEdge1.p1 = leftEnd.end1;
            //leftEnd.endEdge1.p2 = leftEnd.vrts.get(1);
            //leftEnd.endEdge2 = new Edge(leftEnd.vrts.size() - 2, leftEnd.vrts.size() - 1);
            //leftEnd.endEdge2.p1 = leftEnd.vrts.get(leftEnd.vrts.size() - 2);
            //leftEnd.endEdge2.p2 = leftEnd.end2;
            //leftEnd.buildEdges();



            Arc rightStart = new Arc(rightL.center, rightL.radius);
            rightStart.owner = rightL.owner;
            //rightStart.end1 = rightL.end1;
            //rightStart.end2 = cusps[1];
            rightStart.setEndPoints(rightL.end1, cusps[1], true);
            rightStart.prev = rightL.prev;
            rightStart.prev.next = rightStart;
            rightStart.vrts.add(rightStart.end1);
            rightStart.vrts.add(cusps[1]);
            //rightStart.refineLoop(Main.maxEdgeLen, 0.0, true, 1, false);
            //rightStart.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
            //ArcUtil.refineArc(rightStart, Surface.maxEdgeLen, true,1, false);
            ArcUtil.refineArc(rightStart, Surface.maxEdgeLen, false,0, false);
            //rightStart.endEdge1 = new Edge(0, 1);
            //rightStart.endEdge1.p1 = rightStart.end1;
            //rightStart.endEdge1.p2 = rightStart.vrts.get(1);
            //rightStart.endEdge2 = new Edge(rightStart.vrts.size() - 2, rightStart.vrts.size() - 1);
            //rightStart.endEdge2.p1 = rightStart.vrts.get(rightStart.vrts.size() - 2);
            //rightStart.endEdge2.p2 = rightStart.end2;
            //rightStart.buildEdges();



            Arc rightEnd = new Arc(rightL.center, rightL.radius);
            rightEnd.owner = rightL.owner;
            //rightEnd.end1 = cusps[0];
            //rightEnd.end2 = rightL.end2;
            rightEnd.setEndPoints(cusps[0], rightL.end2, true);
            rightEnd.next = rightL.next;
            rightEnd.next.prev = rightEnd;
            rightEnd.vrts.add(cusps[0]);
            rightEnd.vrts.add(rightEnd.end2);
            //rightEnd.refineLoop(Main.maxEdgeLen, 0.0, true, 1, false);
            //rightEnd.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
            //ArcUtil.refineArc(rightEnd, Surface.maxEdgeLen, true,1, false);
            ArcUtil.refineArc(rightEnd, Surface.maxEdgeLen, false,0, false);
            //rightEnd.endEdge1 = new Edge(0, 1);
            //rightEnd.endEdge1.p1 = rightEnd.end1;
            //rightEnd.endEdge1.p2 = rightEnd.vrts.get(1);
            //rightEnd.endEdge2 = new Edge(rightEnd.vrts.size() - 2, rightEnd.vrts.size() - 1);
            //rightEnd.endEdge2.p1 = rightEnd.vrts.get(rightEnd.vrts.size() - 2);
            //rightEnd.endEdge2.p2 = rightEnd.end2;
            //rightEnd.buildEdges();




            //p.v.multiply(-1);
            /*Edge e = bs[0].lines.get(0);
            int i = 0;
            while (!(p.checkPointLocation(e.p1) < 0.0 && p.checkPointLocation(e.p2) > 0.0 || Math.abs(p.checkPointLocation(e.p1)) < 0.001 || Math.abs(p.checkPointLocation(e.p2)) < 0.001)){
                e = e.next;
                if (i > bs[0].lines.size()){
                    rp.valid = false;
                    Main.rectangles.remove(rp);
                    leftL.owner.valid = false;
                    Main.triangles.remove(leftL.owner);
                    rightL.owner.valid = false;
                    Main.triangles.remove(rightL.owner);
                    return;
                }
                i++;
            }
            List<Point> middlevrts = new ArrayList<>();
            middlevrts.add(cusps[0]);
            i = 0;
            while (!(p.checkPointLocation(e.p1) > 0.0 && p.checkPointLocation(e.p2) < 0.0 || Math.abs(p.checkPointLocation(e.p1)) < 0.001 || Math.abs(p.checkPointLocation(e.p2)) < 0.001)){
                middlevrts.add(e.p2);
                e = e.next;
                if (i > bs[0].lines.size()){
                    rp.valid = false;
                    Main.rectangles.remove(rp);
                    leftL.owner.valid = false;
                    Main.triangles.remove(leftL.owner);
                    rightL.owner.valid = false;
                    Main.triangles.remove(rightL.owner);
                    return;
                }
                i++;
            }
            middlevrts.add(cusps[1]);*/

            //Vector toS = Point.subtractPoints(cusps[0], circle).makeUnit();
            //Vector toE = Point.subtractPoints(cusps[1], circle).makeUnit();
            //Vector n = Vector.getNormalVector(toE, toS).makeUnit();
            v1.changeVector(cusps[0], circle).makeUnit(); //toS
            v2.changeVector(cusps[1], circle).makeUnit(); //toE
            n.assignNormalVectorOf(v2, v1).makeUnit();
            middlevrts.clear();
            if (n.dotProduct(circleN) > 0.0){
                //System.out.println("guns n roses");
                ArcUtil.generateCircArc(cusps[0], cusps[1], circle, radius, 2, true, middlevrts);
            } else {
                ArcUtil.generateCircArc(cusps[0], cusps[1], circle, radius, 2, false, middlevrts);
            }
            Arc leftMid = new Arc(circle, radius);
            leftMid.owner = leftL.owner;
            //leftMid.end1 = cusps[0];
            //leftMid.end2 = cusps[1];
            leftMid.setEndPoints(cusps[0], cusps[1], false);
            //leftMid.setNormal(Point.subtractPoints(leftL.owner.sphere.center, circle).makeUnit());
            leftMid.setNormal(v1.changeVector(leftL.owner.sphere.center, circle).makeUnit());
            leftMid.prev = leftStart;
            leftStart.next = leftMid;
            leftMid.next = leftEnd;
            leftEnd.prev = leftMid;
            leftMid.vrts.addAll(middlevrts);
            //leftMid.endEdge1 = new Edge(0, 1);
            //leftMid.endEdge1.p1 = leftMid.end1;
            //leftMid.endEdge1.p2 = leftMid.vrts.get(1);
            //leftMid.endEdge2 = new Edge(leftMid.vrts.size() - 2, leftMid.vrts.size() - 1);
            //leftMid.endEdge2.p1 = leftMid.vrts.get(leftMid.vrts.size() - 2);
            //leftMid.endEdge2.p2 = leftMid.end2;
            //leftMid.mid = middlevrts.get(1);
            //leftMid.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
            ArcUtil.refineArc(leftMid, Surface.maxEdgeLen, false,0, false);
            //Util.reverserOrder(leftMid, true);
            //leftMid.buildEdges();
            //ArcUtil.markShared(leftMid);





            Arc rightMid = ArcUtil.cloneArc(leftMid);
            rightMid.owner = rightL.owner;
            //rightMid.end1 = cusps[0];
            //rightMid.end2 = cusps[1];
            //rightMid.setEndPoints(cusps[0], cusps[1], false);
            rightMid.setNormal(leftMid.normal);
            //rightMid.vrts = ArcUtil.cloneArc(leftMid);
            //Util.reverserOrder(rightMid, true);
            ArcUtil.reverseArc(rightMid, true);
            //ArcUtil.replaceMiddleVertex(rightMid, new Point(leftMid.mid));
            rightMid.baseSubdivision = leftMid.baseSubdivision;
            rightMid.next = rightEnd;
            rightEnd.prev = rightMid;
            rightMid.prev = rightStart;
            rightStart.next = rightMid;
            //rightMid.endEdge1 = new Edge(0, 1);
            //rightMid.endEdge1.p1 = rightMid.end1;
            //rightMid.endEdge1.p2 = rightMid.vrts.get(1);
            //rightMid.endEdge2 = new Edge(rightMid.vrts.size() - 2, rightMid.vrts.size() - 1);
            //rightMid.endEdge2.p1 = rightMid.vrts.get(rightMid.vrts.size() - 2);
            //rightMid.endEdge2.p2 = rightMid.end2;
            rightMid.opposite = leftMid;
            leftMid.opposite = rightMid;

            rightEnd.vrts.remove(0);
            rightEnd.vrts.add(0, rightMid.end2);
            rightEnd.end1 = rightMid.end2;

            rightStart.vrts.remove(rightStart.vrts.size() - 1);
            rightStart.vrts.add(rightMid.end1);
            rightStart.end2 = rightMid.end1;
            //rightMid.buildEdges();




            /*leftStart.buildEdges();
            leftMid.buildEdges();
            leftEnd.buildEdges();

            rightStart.buildEdges();
            rightMid.buildEdges();
            rightEnd.buildEdges();*/
            //ArcUtil.buildEdges(leftStart);
            //ArcUtil.buildEdges(leftMid);
            //ArcUtil.buildEdges(leftEnd);
            //ArcUtil.buildEdges(rightStart);
            //ArcUtil.buildEdges(rightMid);
            //ArcUtil.buildEdges(rightEnd);

            //leftStart.endEdge1.prev = leftL.endEdge1.prev;
            //leftStart.endEdge1.prev.next = leftStart.endEdge1;
            //leftEnd.endEdge2.next = leftL.endEdge2.next;
            //leftEnd.endEdge2.next.prev = leftEnd.endEdge2;
            //rightStart.endEdge1.prev = rightL.endEdge1.prev;
            //rightStart.endEdge1.prev.next = rightStart.endEdge1;
            //rightEnd.endEdge2.next = rightL.endEdge2.next;
            //rightEnd.endEdge2.next.prev = rightEnd.endEdge2;
            //leftMid.endEdge1.prev = leftStart.endEdge2;
            //leftStart.endEdge2.next = leftMid.endEdge1;
            //leftMid.endEdge2.next = leftEnd.endEdge1;
            //leftEnd.endEdge1.prev = leftMid.endEdge2;
            //rightMid.endEdge1.prev = rightStart.endEdge2;
            //rightStart.endEdge2.next = rightMid.endEdge1;
            //rightMid.endEdge2.next = rightEnd.endEdge1;
            //rightEnd.endEdge1.prev = rightMid.endEdge2;

            SphericalPatch left = leftL.owner;
            SphericalPatch right = rightL.owner;
//            if (!left.trimmed){
//                left.trimmed = true;
//                Surface.trimmedTriangles++;
//            }

//            if (!right.trimmed){
//                right.trimmed = true;
//                Surface.trimmedTriangles++;
//            }

            if (left.intersectingPatches.contains(right.id) || right.intersectingPatches.contains(left.id)){
                System.out.println("found al double trimming " + left.id + " " + right.id);
            }

            left.neighbours.add(right);
            right.neighbours.add(left);

            leftStart.owner = leftMid.owner = leftEnd.owner = left;
            rightStart.owner = rightMid.owner = rightEnd.owner = right;
            leftMid.intersecting = true;
            rightMid.intersecting = true;
            Surface.intersectingArcs.add(leftMid);
            Surface.intersectingArcs.add(rightMid);
            left.intersectingPatches.add(right.id);
            right.intersectingPatches.add(left.id);

            left.boundaries.get(0).arcs.clear();
            leftStart.bOwner = leftMid.bOwner = leftEnd.bOwner = left.boundaries.get(0);
            rightStart.bOwner = rightEnd.bOwner = rightMid.bOwner = right.boundaries.get(0);

            Arc l = leftStart;
            do {
                left.boundaries.get(0).arcs.add(l);
                l = l.next;
            } while (l != leftStart);
            right.boundaries.get(0).arcs.clear();
            l = rightStart;
            do {
                right.boundaries.get(0).arcs.add(l);
                l = l.next;
            } while (l != rightStart);
            left.boundaries.get(0).vrts.clear();
            for (Arc lo : left.boundaries.get(0).arcs){
                for (int i = 0; i < lo.vrts.size() - 1; ++i){
                    left.boundaries.get(0).vrts.add(lo.vrts.get(i));
                }
            }
            right.boundaries.get(0).vrts.clear();
            for (Arc lo : right.boundaries.get(0).arcs){
                for (int i = 0; i < lo.vrts.size() - 1; ++i){
                    right.boundaries.get(0).vrts.add(lo.vrts.get(i));
                }
            }

            //left.boundaries.get(0).buildEdges(false);
            //right.b.buildEdges(false);
            ArcUtil.buildEdges(left.boundaries.get(0), false);
            ArcUtil.buildEdges(right.boundaries.get(0), false);

            tp.tr1 = new CuspTriangle();
            tp.tr2 = new CuspTriangle();
            //rp.tr1.base = (leftStart.end1 == rp.convexPatchArcs.get(0).end1) ? rp.convexPatchArcs.get(1) : rp.convexPatchArcs.get(0);
            tp.tr1.base = (Point.distance(leftStart.end1, tp.convexPatchArcs.get(0).end1) < 0.0001) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
            tp.tr1.left = rightEnd;
            tp.tr1.right = leftStart;
            tp.tr1.left.opposite = tp.tr1.right;
            tp.tr1.right.opposite = tp.tr1.left;
            rightEnd.cuspTriangle = leftStart.cuspTriangle = tp.tr1;
            if (rightEnd.vrts.size() != leftStart.vrts.size()){
                System.out.println(" ");
            }
            tp.tr1.cuspPoint = cusps[0];

            tp.tr2.base = (tp.tr1.base == tp.convexPatchArcs.get(0)) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);
            tp.tr2.left = leftEnd;
            tp.tr2.right = rightStart;
            tp.tr2.left.opposite = tp.tr2.right;
            tp.tr2.right.opposite = tp.tr2.left;
            leftEnd.cuspTriangle = rightStart.cuspTriangle = tp.tr2;
            if (leftEnd.vrts.size() != rightStart.vrts.size()){
                System.out.println(" ");
            }
            tp.tr2.cuspPoint = cusps[1];
            //equalizeVertexCount(tp.tr1, tp.tr2);
            leftStart.torus = leftEnd.torus = rightStart.torus = rightEnd.torus = null;
            tp.concavePatchArcs.clear();
        } catch (Exception e){
            System.err.println("tp id: " + tp.id);
            e.printStackTrace();
        }
    }
    private static List<Arc> newArcs = new ArrayList<>();
    private static List<Boundary> newBs = new ArrayList<>();
    private static Plane _p = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));

    public static void processIntersectingArcsOnPatch(Arc arc){
        if (!arc.intersecting || !arc.valid){
            return;
        }
        //List<Point> pointOfIntersection = new ArrayList<>();
        intersectionPoints.clear();
        Point lastPoI = null;
        boolean found = false;
        SphericalPatch sp = arc.owner;
        //Vector u1 = Point.subtractPoints(arc.end1, arc.center).makeUnit();
        //Vector u2 = Point.subtractPoints(arc.end2, arc.center).makeUnit();
        //Vector n1 = Point.subtractPoints(sp.sphere.center, arc.center).makeUnit();
        Vector u1 = v1.changeVector(arc.end1, arc.center).makeUnit();
        Vector u2 = v2.changeVector(arc.end2, arc.center).makeUnit();
        Vector n1 = v3.changeVector(sp.sphere.center, arc.center).makeUnit();
        //Plane p1 = new Plane(arc.center, n1);
        p1.redefine(arc.center, n1);
        for (int i = 0; i < sp.boundaries.size(); ++i){
            Boundary b = sp.boundaries.get(i);
            for (int j = 0; j < b.arcs.size(); ++j){
                Arc k = b.arcs.get(j);
                if (k == arc || k == arc.prev || k == arc.next){
                    continue;
                }
                boolean allInside = true;//k.vrts.stream().allMatch(p -> p1.checkPointLocation(p) > 0.0);
                for (int l = 0; l < k.vrts.size(); ++l){
                    if (!(p1.checkPointLocation(k.vrts.get(l)) > 0.0)){
                        allInside = false;
                        break;
                    }
                }
                if (allInside){
                    continue;
                }
                //Vector v1 = Point.subtractPoints(k.end1, k.center).makeUnit();
                //Vector v2 = Point.subtractPoints(k.end2, k.center).makeUnit();
                v1.changeVector(k.end1, k.center).makeUnit();
                v2.changeVector(k.end2, k.center).makeUnit();
                //Plane p2 = new Plane(k.center, v1, v2);
                p2.redefine(k.center, v1, v2);
                //Vector dir = new Vector(0, 0, 0);
                //Point pint = new Point(0, 0, 0);
                if (Plane.getIntersectionLine(p1, p2, v1, pInt)){
                    //Vector hypo = Point.subtractPoints(arc.center, pint);
                    hypo.changeVector(arc.center, pInt);
                    v1.makeUnit();
                    double odvesna = v1.dotProduct(hypo);
                    double dist = Math.sqrt(hypo.dotProduct(hypo) - Math.pow(odvesna, 2));
                    if (dist - arc.radius < 0.0){
                        //Point midTetiva = Point.translatePoint(pint, Vector.scaleVector(dir, odvesna));
                        midOfChord.assignTranslation(pInt, v1.multiply(odvesna));
                        double odv2 = Math.sqrt(Math.pow(arc.radius, 2) - Math.pow(dist, 2));
                        //Point in1 = Point.translatePoint(midOfChord, v1.makeUnit().multiply(odv2));//Vector.scaleVector(dir, odv2));
                        //Point in2 = Point.translatePoint(midOfChord, v1.multiply(-1.0));//Vector.scaleVector(dir, -odv2));
                        in1.assignTranslation(midOfChord, v1.makeUnit().multiply(odv2));
                        in2.assignTranslation(midOfChord, v1.multiply(-1.0));
                            /*if ((k.isInside(in1) || k.isInside(in2)) && (cpl.isInside(in1) || cpl.isInside(in2))) {
                                System.out.println("possible intermezzo for " + cpl.owner.id);
                            }*/
                        if ((k.isInside(in1) && arc.isInside(in1)) || (k.isInside(in2) && arc.isInside(in2))){
                            //System.out.println("INTERMEZZO for " + arc.owner.id);
                            found = true;
                        }
                        if (k.isInside(in1) && arc.isInside(in1)){
                                /*if (lastPoI == null || Point.distance(lastPoI, in1) > 0.001){
                                    lastPoI = in1;
                                    pointOfIntersection.add(in1);
                                }*/
                            boolean foundSimilar = false;
                            for (int l = 0; l < intersectionPoints.size(); ++l){
                                if (Point.distance(in1, intersectionPoints.get(l)) < 0.001){
                                    foundSimilar = true;
                                    break;
                                }
                            }
                            //if (!intersectionPoints.stream().anyMatch(p -> Point.distance(p, in1) < 0.001)){
                            //    intersectionPoints.add(new Point(in1));
                            //}
                            if (!foundSimilar){
                                intersectionPoints.add(new Point(in1));
                            }
                        }
                        if (k.isInside(in2) && arc.isInside(in2)){
                                /*if (lastPoI == null || Point.distance(lastPoI, in2) > 0.001){
                                    lastPoI = in2;
                                    pointOfIntersection.add(in2);
                                }*/
                            //if (!intersectionPoints.stream().anyMatch(p -> Point.distance(p, in2) < 0.001)){
                            //    intersectionPoints.add(new Point(in2));
                            //}
                            boolean foundSimilar = false;
                            for (int l = 0; l < intersectionPoints.size(); ++l){
                                if (Point.distance(in2, intersectionPoints.get(l)) < 0.001){
                                    foundSimilar = true;
                                    break;
                                }
                            }
                            if (!foundSimilar){
                                intersectionPoints.add(new Point(in2));
                            }
                        }
                    }
                }
            }
        }
        if (found && intersectionPoints.size() == 2) {
            for (int i = 0; i < sp.boundaries.size(); ++i){
                Boundary b = sp.boundaries.get(i);
                for (int j = 0; j < b.arcs.size(); ++j){
                    Arc a = b.arcs.get(j);
                    a.valid = false;
                }
            }
            /*Point e1 = intersectionPoints.get(0);
            Point e2 = Point.translatePoint(e1, v1.changeVector(arc.normal).multiply(5));//Point.translatePoint(e1, Vector.scaleVector(arc.normal, 5));
            Point f1 = intersectionPoints.get(1);
            Point f2 = Point.translatePoint(f1, Vector.scaleVector(arc.normal, 5));
            Edge ed1 = new Edge(0, 0);
            ed1.p1 = e1;
            ed1.p2 = e2;
            Edge ed2 = new Edge(0, 0);
            ed2.p1 = f1;
            ed2.p2 = f2;*/
            //intersectionPointsList<Boundary> newBs = new ArrayList<>();
            newBs.clear();
            Boundary b = new Boundary();
            b.patch = sp;
            Arc start = arc.next.next;
            Arc a = start;
            do {
                //Plane p = new Plane(a.center, a.normal);
                _p.redefine(a.center, a.normal);
                boolean trim = false;
                Point in = null;
                for (int j = 0; j < intersectionPoints.size(); ++j){
                    Point i = intersectionPoints.get(j);
                    if (Math.abs(_p.checkPointLocation(i)) < 0.001 && a.isInside(i)){
                        trim = true;
                        if (in == null || Point.distance(in, a.end1) - Point.distance(i, a.end1) > 0.0){
                            in = i;
                        }
                    }
                }
                if (trim){
                        /*a.vrts.clear();
                        a.vrts.add(a.end1);
                        a.vrts.add(in);
                        a.valid = true;
                        a.setEndPoints(a.end1, in, false);*/
                    Arc newA2 = new Arc(a.center, a.radius);
                    newA2.setEndPoints(a.end1, in, false);
                    newA2.setNormal(a.normal);
                    newA2.vrts.add(a.end1);
                    newA2.vrts.add(in);
                    newA2.valid = true;
                    //newA2.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    ArcUtil.refineArc(newA2, Surface.maxEdgeLen, false,0, false);
                    Arc newA = new Arc(arc.center, arc.radius);
                    newA.setNormal(arc.normal);
                    newA.vrts.add(in);
                    newA.vrts.add(arc.end2);
                    //newA.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    newA.setEndPoints(in, arc.end2, false);
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false,0, false);

                    newA.prev = newA2;
                    newA2.next = newA;
                    newA.next = arc.next;
                    newA.next.prev = newA;
                    b.arcs.add(newA2);
                    b.arcs.add(newA);
                    a.intersecting = false;
                    //a.buildEdges();
                    a = newA;
                    //a.buildEdges();
                    a.intersecting = false;
                    a.valid = true;
                    intersectionPoints.remove(in);
                } else {
                    a.valid = true;
                    b.arcs.add(a);
                }
                a = a.next;
            } while (a != start);
            //b.buildEdges(true);
            ArcUtil.buildEdges(b, true);
            newBs.add(b);
            b = new Boundary();
            b.patch = sp;
            newArcs.clear();
            //List<Arc> newArcs = b.arcs;//new ArrayList<>();
            start = arc.prev.prev;
            a = start;
            do {
                //Plane p = new Plane(a.center, a.normal);
                _p.redefine(a.center, a.normal);
                boolean trim = false;
                Point in = null;
                for (int j = 0; j < intersectionPoints.size(); ++j){
                    Point i = intersectionPoints.get(j);
                    if (Math.abs(_p.checkPointLocation(i)) < 0.001 && a.isInside(i)){
                        trim = true;
                        if (in == null || Point.distance(in, a.end2) - Point.distance(i, a.end2) > 0.0) {
                            in = i;
                        }
                    }
                }
                if (trim){
                    a.vrts.clear();
                    a.vrts.add(in);
                    a.vrts.add(a.end2);
                    //a.mid = null;
                    //a.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    a.setEndPoints(in, a.end2, false);
                    ArcUtil.refineArc(a, Surface.maxEdgeLen, false,0, false);
                    a.baseSubdivision = ArcUtil.getSubdivisionLevel(a);

                    a.valid = true;
                    arc.vrts.clear();
                    arc.vrts.add(arc.end1);
                    arc.vrts.add(in);
                    //arc.mid = null;
                    //arc.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    arc.setEndPoints(arc.end1, in, false);
                    ArcUtil.refineArc(arc, Surface.maxEdgeLen, false,0, false);
                    arc.baseSubdivision = ArcUtil.getSubdivisionLevel(arc);

                    a.prev = arc;
                    arc.next = a;
                    arc.valid = true;
                    //a.buildEdges();
                    //cpl.buildEdges();
                    newArcs.add(a);
                    newArcs.add(arc);
                    intersectionPoints.remove(in);
                    a.intersecting = false;
                    a = arc;
                    a.intersecting = false;
                } else {
                    a.valid = true;
                    newArcs.add(a);
                }
                a = a.prev;
            } while (a != start);
            for (int i = newArcs.size() - 1; i >= 0; --i){
                b.arcs.add(newArcs.get(i));
            }
            //b.buildEdges(true);
            ArcUtil.buildEdges(b, true);
            newBs.add(b);
            sp.boundaries.clear();
            sp.boundaries.addAll(newBs);
        }
        intersectionPoints.clear();
        lastPoI = null;
    }

    public static double computeIntersectionCircle(Point probe1, Point probe2, Point result, double probeRadius){
        //Vector halfway = Point.subtractPoints(probe1, probe2).multiply(0.5f);
        v1.changeVector(probe1, probe2).multiply(0.5f);
        //Point center = Point.translatePoint(probe2, halfway);
        point.assignTranslation(probe2, v1);
        result.x = point.x;
        result.y = point.y;
        result.z = point.z;
        return Math.sqrt(Math.pow(probeRadius, 2) - Math.pow(v1.sqrtMagnitude(), 2));
    }

    public static Point computeCusp(Point probe, Sphere a1, Sphere a2){
        //Vector axis = Point.subtractPoints(a2.center, a1.center).makeUnit();
        //Vector a1toprobe = Point.subtractPoints(probe, a1.center);
        v1.changeVector(a2.center, a1.center).makeUnit(); //axis
        v2.changeVector(probe, a1.center).makeUnit(); //a1toprobe
        double atomToProbeLength = a1.radius + SesConfig.probeRadius;
        v2.makeUnit();
        double alpha = Math.acos(v1.dotProduct(v2));
        double beta = Math.asin(((atomToProbeLength) * Math.sin(alpha)) / SesConfig.probeRadius);
        double gama = Math.PI - alpha - beta;
        double atomToCusp = (SesConfig.probeRadius * Math.sin(gama)) / Math.sin(alpha);
        v1.multiply(atomToCusp);
        return Point.translatePoint(a1.center, v1);
    }

    public static void processIntersectingConcavePatches(){
        for (SphericalPatch sp : Surface.triangles){
            trimConcavePatch(sp);
        }
        planes.clear();
    }

    public static void processSelfIntersectingTori(){
        for (ToroidalPatch tp : Surface.selfIntersectingRects){
            PatchUtil.torProcessSelfIntersection(tp);
        }
    }

    public static void processSelfIntersectingConcavePatches() {
        try {
            for (Arc cpl : Surface.intersectingArcs) {
                PatchUtil.trimSelfIntersectingPatch(cpl);
            }
        } catch(Exception e){
            e.printStackTrace();
        }
    }

    private static int torId = 3500;

    private static Boundary boundaryAlgorithm1(Plane circle, double radius, Point in1, Arc in1Arc, Point in2, Arc in2Arc, List<Boundary> newBS, SphericalPatch sp){
        Boundary b = new Boundary();
        b.patch = sp;
        Arc newA;
        /*if (Point.distance(in1, in1Arc.end2) < 0.001){
            newA = in1Arc;
        } else {*/
            newA = new Arc(in1Arc.center, in1Arc.radius);
            newA.setEndPoints(in1Arc.end1, in1, true);
            newA.vrts.add(in1Arc.end1);
            newA.vrts.add(in1);
        if (in1Arc.cuspTriangle != null){
            int subdLevel = ArcUtil.getSubdivisionLevel(in1Arc);
            newA.cuspTriangle = in1Arc.cuspTriangle;
            newA.torus = in1Arc.torus;
            if (newA.cuspTriangle.left == in1Arc){
                newA.cuspTriangle.left = newA;
            } else {
                newA.cuspTriangle.right = newA;
            }
            ArcUtil.refineArc(newA, 0, true, subdLevel, false);
        } else {
            ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
        }
        //}
        b.arcs.add(newA);
        Point bp1 = newA.end2;
        Point bp2 = (Point.distance(in2Arc.end1, in2) < 0.001) ? in2Arc.end1 : in2;
        newA = new Arc(circle.p, radius);
        newA.setEndPoints(bp1, bp2, false);
        newA.setNormal(circle.v);
        newA.vrts.add(bp1);
        newA.vrts.add(bp2);
        ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
        b.arcs.add(newA);
        /*if (bp2 == in2Arc.end1){
            newA = in2Arc;
        } else {*/
            newA = new Arc(in2Arc.center, in2Arc.radius);
            newA.setEndPoints(bp2, in2Arc.end2, true);
            newA.vrts.add(bp2);
            newA.vrts.add(in2Arc.end2);

            if (in2Arc.cuspTriangle != null){
                int subdLevel = ArcUtil.getSubdivisionLevel(in2Arc);
                newA.cuspTriangle = in2Arc.cuspTriangle;
                newA.torus = in2Arc.torus;
                if (newA.cuspTriangle.left == in2Arc){
                    newA.cuspTriangle.left = newA;
                } else {
                    newA.cuspTriangle.right = newA;
                }
                ArcUtil.refineArc(newA, 0, true, subdLevel, false);
            } else {
                ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
            }
        //}
        b.arcs.add(newA);
        Arc a = in2Arc.next;
        while (a != in1Arc){
            newA = ArcUtil.dbgCloneArc(a);
            if (a.cuspTriangle != null){
                newA.cuspTriangle = a.cuspTriangle;
                newA.torus = a.torus;
                if (newA.cuspTriangle.left == a){
                    newA.cuspTriangle.left = newA;
                } else {
                    newA.cuspTriangle.right = newA;
                }
            }
            b.arcs.add(newA);
            a = a.next;
        }
        ArcUtil.buildEdges(b, true);
        newBS.add(b);
        //int kratky = shortArcs(b);
        return in1Arc.bOwner;
    }
    private static List<Boundary> newBS = new ArrayList<>();
    private static List<Boundary> toRemove = new ArrayList<>();
    private static List<Boundary> toRemove2 = new ArrayList<>();
    private static void boundaryAlgorithm2(Plane circle, double radius, Point in1, Arc a1, Point in2, Arc a2, List<Point> intersectionPoints, List<Point> usedPoints, List<Boundary> toRemove, List<Boundary> newBS, SphericalPatch sp, int otherPatch){
        Point pStart = in1;
        //List<Boundary> toRemove2 = new ArrayList<>();
        toRemove2.clear();
        Boundary b = new Boundary();
        Point newArcCenter = new Point(circle.p);
        b.patch = sp;
        Arc start = a1;
        Arc a = a2;

        //boolean toBridge = (a1 != a2 || circle.checkPointLocation(a1.end1) > 0.0);
        boolean toBridge = isOptimal(in1, a1, circle);
        if (toBridge && !isOptimal(in1, a1, circle)){
            int o = 8;
        }
        /*if (intersectionPoints.size() > 2 && a1.bOwner == a2.bOwner && Vector.getNormalVector(Point.subtractPoints(in1, circle.p).makeUnit(), Point.subtractPoints(in2, circle.p).makeUnit()).makeUnit().dotProduct(circle.v) < 0.0){
            int c = 3;
            toBridge = false;
        }*/
        boolean forceContinue = false;
        boolean arcExit = false;
        int it = 0;
        //boolean forceBridge = (circle.checkPointLocation(a1.end1) < 0.0 && circle.checkPointLocation(a1.end2) < 0.0);
        do {
            if (toBridge){ //|| forceBridge){
                Arc newA = null; //retrieveBridgeArc(sp.id, otherPatch, in2, in1);
                if (newA == null) {
                    newA = new Arc(newArcCenter, radius);
                    newA.setEndPoints(in1, in2, false);
                    newA.setNormal(circle.v);
                    newA.vrts.add(in1);
                    newA.vrts.add(in2);
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    //putNewBridgeArc(sp.id, otherPatch, newA);
                } else {
                    Arc temp = newA;
                    newA = ArcUtil.cloneArc(temp);
                    ArcUtil.reverseArc(newA, true);
                    newA.opposite = temp;
                    temp.opposite = newA;
                    /*newA = new Arc(circle.p, radius);
                    newA.setEndPoints(in1, in2, false);
                    newA.setNormal(circle.v);
                    newA.vrts.add(in1);
                    newA.vrts.add(in2);
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);*/
                }
                usedPoints.add(in1);
                usedPoints.add(in2);
               // ArcUtil.markShared(newA);
                b.arcs.add(newA);
                toBridge = false;
                in1 = in2;
                a1 = a2;
                in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                a2 = in2.arc;//ArcUtil.findContainingArc(in2, circle, sp, null);
                while (a2.bOwner != a1.bOwner && !a2.bOwner.nestedBoundaries.contains(a1.bOwner)){
                    in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in2, false, circle.p, circle.v, true);
                    a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                }
                a = a1;
                forceContinue = (in1 != pStart);
                arcExit = true;
                if (!toRemove2.contains(a1.bOwner)) {
                    toRemove2.add(a1.bOwner);
                    a1.bOwner.mergeSplit.add(b);
                    b.mergeSplit.add(a1.bOwner);
                }
                if (!toRemove2.contains(a2.bOwner)) {
                    toRemove2.add(a2.bOwner);
                    a2.bOwner.mergeSplit.add(b);
                    b.mergeSplit.add(a2.bOwner);
                }
                            /*if (forceBridge){
                                forceBridge = false;
                                toBridge = true;
                            }*/
            } else if (a1 == a2 && ArcUtil.getOrder(a1, in1, in2) < 0) {
                if (ArcUtil.getOrder(a1, in1, in2) < 0) {
                    Arc newA = new Arc(a1.center, a1.radius);
                    newA.setEndPoints(in1, in2, false);
                    newA.setNormal(a1.normal);
                    newA.vrts.add(in1);
                    newA.vrts.add(in2);
                    usedPoints.add(in2);
                    if (a1.torus != null){
                        System.out.println("found torus 1");
                    }
                    if (a1.opposite != null){
                        newA.opposite = a1.opposite;
                        newA.opposite.opposite = newA;
                        //ArcUtil.refineArc(newA, 0, true, ArcUtil.getSubdivisionLevel(newA.opposite), false);
                        if (a1.torus != null){
                            Arc finalA = a1;
                            a1.torus.concavePatchArcs.removeIf(new Predicate<Arc>() {
                                @Override
                                public boolean test(Arc arc) {
                                    return arc.id == finalA.id;
                                }
                            });
                            newA.torus = a1.torus;
                            newA.torus.concavePatchArcs.add(newA);
                        }
                    } else {
                        //ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    }
                    if (a1.cuspTriangle != null){
                        System.out.println("found arc of cusp 1");
                    }
                    /*if (a1.torus != null){
                        if (a1.torus.tr1 != null){
                            if (a1 == a1.torus.tr1.left){
                                a1.torus.tr1.left = newA;
                            } else if (a1 == a1.torus.tr1.right){
                                a1.torus.tr1.right = newA;
                            } else if (a1 == a1.torus.tr2.left){
                                a1.torus.tr2.left = newA;
                            } else if (a1 == a1.torus.tr2.right){
                                a1.torus.tr2.right = newA;
                            }
                        } else {
                            if (a1.torus.concavePatchArcs.get(0) == a1){
                                a1.torus.concavePatchArcs.remove(a1);
                                a1.torus.concavePatchArcs.add(newA);
                            } else {
                                a1.torus.concavePatchArcs.remove(a1);
                                a1.torus.concavePatchArcs.add(newA);
                            }
                        }
                        newA.torus = a1.torus;
                    }*/
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    toBridge = true;
                    in1 = in2;
                    in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                    a2 = in2.arc;//ArcUtil.findContainingArc(in2, circle, sp, null);
                    while (a2.bOwner != a1.bOwner && !a2.bOwner.nestedBoundaries.contains(a1.bOwner)) {
                        in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in2, false, circle.p, circle.v, true);
                        a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                    }
                    a = a2;
                    b.arcs.add(newA);
                    forceContinue = false;
                    //it is necessary to set a...cause now a==start = true -> premature end
                }
            } else if (a == a1 && arcExit){
                if (Point.distance(in1, a1.end2) > 0.0015) {
                    Arc newA = new Arc(a1.center, a1.radius);
                    newA.setEndPoints(in1, a1.end2, false);
                    newA.setNormal(a.normal);
                    newA.vrts.add(in1);
                    newA.vrts.add(a1.end2);
                    //ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    /*if (a.cuspTriangle != null){
                        System.out.println("found arc of cusp 2");
                    }*/
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    if (a1.torus != null){
                        System.out.println("found torus 2");
                    }
                    if (a.opposite != null){
                        newA.opposite = a.opposite;
                        newA.opposite.opposite = newA;
                        //ArcUtil.refineArc(newA, 0, true, ArcUtil.getSubdivisionLevel(newA.opposite), false);
                        if (a.torus != null){
                            newA.torus = a.torus;
                            Arc finalA = a;
                            a.torus.concavePatchArcs.removeIf(new Predicate<Arc>() {
                                @Override
                                public boolean test(Arc arc) {
                                    return arc.id == finalA.id;
                                }
                            });
                            newA.torus.concavePatchArcs.add(newA);
                        } else if (a.cuspTriangle != null){
                            newA.cuspTriangle = a.cuspTriangle;
                            if (a == a.cuspTriangle.left){
                                newA.cuspTriangle.left = newA;
                                newA.cuspTriangle.right.opposite = newA;
                                newA.opposite = newA.cuspTriangle.right;
                            } else {
                                newA.cuspTriangle.right = newA;
                                newA.cuspTriangle.left.opposite = newA;
                                newA.opposite = newA.cuspTriangle.left;
                            }
                        }
                    } else {
                        //ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    }

                    /*if (a.cuspTriangle != null){
                        int subdLevel = ArcUtil.getSubdivisionLevel(a);
                        newA.cuspTriangle = a.cuspTriangle;
                        newA.torus = a.torus;
                        if (newA.cuspTriangle.left == a){
                            newA.cuspTriangle.left = newA;
                        } else {
                            newA.cuspTriangle.right = newA;
                        }
                        ArcUtil.refineArc(newA, 0, true, subdLevel, false);
                    } else {
                        ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    }
                    if (a1.torus != null){
                        if (a1.torus.tr1 != null){
                            if (a1 == a1.torus.tr1.left){
                                a1.torus.tr1.left = newA;
                            } else if (a1 == a1.torus.tr1.right){
                                a1.torus.tr1.right = newA;
                            } else if (a1 == a1.torus.tr2.left){
                                a1.torus.tr2.left = newA;
                            } else if (a1 == a1.torus.tr2.right){
                                a1.torus.tr2.right = newA;
                            }
                        } else {
                            if (a1.torus.concavePatchArcs.get(0) == a1){
                                a1.torus.concavePatchArcs.remove(a1);
                                a1.torus.concavePatchArcs.add(newA);
                            } else {
                                a1.torus.concavePatchArcs.remove(a1);
                                a1.torus.concavePatchArcs.add(newA);
                            }
                        }
                        newA.torus = a1.torus;
                    }*/
                    b.arcs.add(newA);
                }
                a = a.next;
                forceContinue = false;
                arcExit = false;
            } else if (a == a2 && !arcExit){
                if (Point.distance(a.end1, in2) > 0.0015) {
                    Arc newA = new Arc(a.center, a.radius);
                    newA.setEndPoints(a.end1, in2, false);
                    newA.setNormal(a.normal);
                    newA.vrts.add(a.end1);
                    newA.vrts.add(in2);
                    /*if (a.cuspTriangle != null){
                        System.out.println("found arc of cusp 3");
                    }*/
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    if (a.opposite != null){
                        newA.opposite = a.opposite;
                        newA.opposite.opposite = newA;
                        //ArcUtil.refineArc(newA, 0, true, ArcUtil.getSubdivisionLevel(newA.opposite), false);
                        if (a.torus != null){//this should not happen
                            newA.torus = a.torus;
                            Arc finalA = a;
                            a.torus.concavePatchArcs.removeIf(new Predicate<Arc>() {
                                @Override
                                public boolean test(Arc arc) {
                                    return arc.id == finalA.id;
                                }

                            });
                            newA.torus.concavePatchArcs.add(newA);
                        } else if (a.cuspTriangle != null){
                            newA.cuspTriangle = a.cuspTriangle;
                            if (a == a.cuspTriangle.left){
                                newA.cuspTriangle.left = newA;
                                newA.cuspTriangle.right.opposite = newA;
                                newA.opposite = newA.cuspTriangle.right;
                            } else {
                                newA.cuspTriangle.right = newA;
                                newA.cuspTriangle.left.opposite = newA;
                                newA.opposite = newA.cuspTriangle.left;
                            }
                        }
                    } else {

                    }
                    /*if (a.cuspTriangle != null){
                        int subdLevel = ArcUtil.getSubdivisionLevel(a);
                        newA.cuspTriangle = a.cuspTriangle;
                        newA.torus = a.torus;
                        if (newA.cuspTriangle.left == a){
                            newA.cuspTriangle.left = newA;
                        } else {
                            newA.cuspTriangle.right = newA;
                        }
                        ArcUtil.refineArc(newA, 0, true, subdLevel, false);
                    } else {
                        ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    }
                    if (a.torus != null){
                        if (a.torus.tr1 != null){
                            if (a == a.torus.tr1.left){
                                a.torus.tr1.left = newA;
                                a.cuspTriangle = a.torus.tr1;
                            } else if (a == a.torus.tr1.right){
                                a.torus.tr1.right = newA;
                                a.cuspTriangle = a.torus.tr1;
                            } else if (a == a.torus.tr2.left){
                                a.torus.tr2.left = newA;
                                a.cuspTriangle = a.torus.tr2;
                            } else if (a == a.torus.tr2.right){
                                a.torus.tr2.right = newA;
                                a.cuspTriangle = a.torus.tr2;
                            }
                        } else {
                            if (a.torus.concavePatchArcs.get(0) == a){
                                a.torus.concavePatchArcs.remove(a);
                                a.torus.concavePatchArcs.add(newA);
                            } else {
                                a.torus.concavePatchArcs.remove(a);
                                a.torus.concavePatchArcs.add(newA);
                            }
                        }
                        newA.torus = a.torus;
                    }*/
                    b.arcs.add(newA);
                }
                a = a.next;
                forceContinue = false;
                toBridge = true;
                in1 = in2;
                a1 = a2;
                in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                a2 = in2.arc;//ArcUtil.findContainingArc(in2, circle, sp, null);
                while (a2.bOwner != a1.bOwner && !a2.bOwner.nestedBoundaries.contains(a1.bOwner)){
                    in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in2, false, circle.p, circle.v, true);
                    a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                }
            }else {
                Arc newA = ArcUtil.dbgCloneArc(a);
                if (a.opposite != null){
                    newA.opposite = a.opposite;
                    newA.opposite.opposite = newA;
                    //ArcUtil.refineArc(newA, 0, true, ArcUtil.getSubdivisionLevel(newA.opposite), false);
                    if (a.torus != null){
                        newA.torus = a.torus;
                        //Arc finalA = a;
                        //a.torus.concavePatchArcs.removeIf(new Predicate<Arc>() {
                        //    @Override
                        //    public boolean test(Arc arc) {
                        //        return arc.id == finalA.id;
                        //    }
                        //});
                        int l = 0;
                        while (l < a.torus.concavePatchArcs.size()){
                            if (a.torus.concavePatchArcs.get(l).id == a.id){
                                a.torus.concavePatchArcs.remove(l);
                                break;
                            }
                            l++;
                        }
                        newA.torus.concavePatchArcs.add(newA);
                    } else if (a.cuspTriangle != null){
                        newA.cuspTriangle = a.cuspTriangle;
                        if (a == a.cuspTriangle.left){
                            newA.cuspTriangle.left = newA;
                            newA.cuspTriangle.right.opposite = newA;
                            newA.opposite = newA.cuspTriangle.right;
                        } else {
                            newA.cuspTriangle.right = newA;
                            newA.cuspTriangle.left.opposite = newA;
                            newA.opposite = newA.cuspTriangle.left;
                        }
                    }
                } else {
                    //ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                }
                /*if (a.cuspTriangle != null){
                    newA.cuspTriangle = a.cuspTriangle;
                    newA.torus = a.torus;
                    if (newA.cuspTriangle.left == a){
                        newA.cuspTriangle.left = newA;
                    } else {
                        newA.cuspTriangle.right = newA;
                    }
                }
                if (a.torus != null){
                    if (a.torus.tr1 != null){
                        if (a == a.torus.tr1.left){
                            a.torus.tr1.left = newA;
                            a.cuspTriangle = a.torus.tr1;
                        } else if (a == a.torus.tr1.right){
                            a.torus.tr1.right = newA;
                            a.cuspTriangle = a.torus.tr1;
                        } else if (a == a.torus.tr2.left){
                            a.torus.tr2.left = newA;
                            a.cuspTriangle = a.torus.tr2;
                        } else if (a == a.torus.tr2.right){
                            a.torus.tr2.right = newA;
                            a.cuspTriangle = a.torus.tr2;
                        }
                    } else {
                        if (a.torus.concavePatchArcs.get(0) == a){
                            a.torus.concavePatchArcs.remove(a);
                            a.torus.concavePatchArcs.add(newA);
                        } else {
                            a.torus.concavePatchArcs.remove(a);
                            a.torus.concavePatchArcs.add(newA);
                        }
                    }
                    newA.torus = a.torus;
                }*/
                b.arcs.add(newA);
                a = a.next;
            }
        } while (forceContinue || in1 != pStart);
        ArcUtil.buildEdges(b, true);
        newBS.add(b);
        //int kratky  = shortArcs(b);
        for (int i = 0; i < toRemove2.size(); ++i){
            Boundary b_ = toRemove2.get(i);
            if (!toRemove.contains(b_)){
                toRemove.add(b_);
            }
        }
        //return toRemove;
        //return a1.bOwner;
    }

    private static int id = -1002;
    private static void generateNewBoundaries2(SphericalPatch sp, List<Point> intersectionPs, Plane circle, double radius, int otherPatch, boolean force){
        try {
            if (intersectionPs.size() % 2 == 1){
                return;
            }
            //List<Point> intersectionPoints = new ArrayList<>(intersectionPs);
            //List<Point> usedPoints = new ArrayList<>();
            usedPoints.clear();
            toRemove.clear();
            newBS.clear();
            int count = intersectionPoints.size();
            if (intersectionPoints.size() > 1) {
                //List<Boundary> newBS = new ArrayList<>();
                //List<Boundary> toRemove = new ArrayList<>();
                int i = 0;
                while (i < count) {
                    Point in1 = findOptimalPoint(intersectionPoints, usedPoints, sp, circle);
                    if (in1 == null){
                        break;
                    }
                    Point in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                    Point pStart = in1;
                    if (in2 == null){
                        int dasf = 3;
                    }
                    Arc a1 = in1.arc;//ArcUtil.findContainingArc(in1, circle, sp, null);
                    Arc a2 = in2.arc;//ArcUtil.findContainingArc(in2, circle, sp, null);
                    if (a1 == null || a2 == null){
                        int b = 433;
                    }
                    final Point p1 = in1;
                    final Point p2 = in2;

                    if (a1.bOwner != a2.bOwner && !a1.bOwner.nestedBoundaries.contains(a2.bOwner)){
                        return;
                    }
                    if (a1.bOwner != a2.bOwner && a1.bOwner.nestedBoundaries.contains(a2.bOwner)) {
                        //System.out.println("about to merge nested boundaries");
                        int c = 32;
                    }

                    if (Point.distance(a1.end1, p1) < 0.002){
                        if (circle.checkPointLocation(a1.prev.end1) > 0.0){
                            a1 = a1.prev;
                            in1.arc = a1;
                        }
                    }

                    if (Point.distance(a2.end2, p2) < 0.002){
                        if (circle.checkPointLocation(a2.next.end2) > 0.0){
                            a2 = a2.next;
                            in2.arc = a2;
                        }
                    }

                    //int spl = patchSplit(intersectionPoints, in1, in2, sp, circle);
                    //List<Point> ps = (intersectionPoints.size() > 2) ?  patchSplit(intersectionPoints, in1, in2, sp, circle) : intersectionPoints;
                    List<Point> ps = (intersectionPoints.size() > 2) ? getUsablePoints(intersectionPoints, in1, in2, sp, circle) : intersectionPoints;

                    //boolean inside = a1.vrts.stream().allMatch(p -> (Point.distance(p, p1) < 0.001) || circle.checkPointLocation(p) > 0.0 || circle.distanceFromPlane(p) < 0.001) &&
                    //        a1.next.vrts.stream().allMatch(p -> Point.distance(p, p1) < 0.001 || circle.checkPointLocation(p) > 0.0 || circle.distanceFromPlane(p) < 0.001) &&
                    //        a1.prev.vrts.stream().allMatch(p -> Point.distance(p, p1) < 0.001 || circle.checkPointLocation(p) > 0.0 || circle.distanceFromPlane(p) < 0.001) &&
                    //        a2.vrts.stream().allMatch(p -> Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0 || circle.distanceFromPlane(p) < 0.001) &&
                    //        a2.next.vrts.stream().allMatch(p -> Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0 || circle.distanceFromPlane(p) < 0.001) &&
                    //        a2.prev.vrts.stream().allMatch(p -> Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0 || circle.distanceFromPlane(p) < 0.001);
                    boolean inside = false;
                    if (arcInsideHyperspace(a1, circle, p1) && arcInsideHyperspace(a1.next, circle, p1) && arcInsideHyperspace(a1.prev, circle, p1) &&
                            arcInsideHyperspace(a2, circle, p2) && arcInsideHyperspace(a2.next, circle, p2) && arcInsideHyperspace(a2.prev, circle, p2)){
                        inside = true;
                    }
                    if (!force && inside){
                        return;
                    }

                    if (Point.distance(in1, a1.end2) < 0.001 && Point.distance(in2, a2.end2) < 0.001){
                        boolean allInside = true;
                        for (int j = 0; j < a1.bOwner.arcs.size(); ++j){
                            Arc a = a1.bOwner.arcs.get(j);
                            //allInside = allInside && a.vrts.stream().allMatch(p -> (Point.distance(p, p1) < 0.001 || Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0));
                            for (int l = 0; l < a.vrts.size(); ++l){
                                Point point = a.vrts.get(l);
                                if (!(Point.distance(point, p1) < 0.001 || Point.distance(point, p2) < 0.001 || circle.checkPointLocation(point) > 0.0)){
                                    allInside = false;
                                    break;
                                }
                            }
                        }
                        if (allInside){
                            return;
                        }
                    }

                    if (a1.next == a2.prev && Point.distance(in1, a1.end2) < 0.001 && Point.distance(in2, a2.end1) < 0.001){
                        //System.err.println(sp.id + " a1n == a2p " + intersectionPoints.size());
                        boolean allInside = true;
                        for (int j = 0; j < a1.next.vrts.size(); ++j){
                            Point point = a1.next.vrts.get(j);
                            if (!(Point.distance(point, p1) < 0.001 || Point.distance(point, p2) < 0.001 || circle.checkPointLocation(point) > 0.0)){
                                allInside = false;
                                break;
                            }
                        }
                        if (allInside){
                            return;
                        }
                        //if (a1.next.vrts.stream().allMatch(p -> (Point.distance(p, p1) < 0.001 || Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0))){
                        //    return;
                        //}
                    }


                    //System.out.println("going to modify " + sp.id);
                    /*if (spl < 0){
                     i += 2;
                     usedPoints.add(in1);
                     usedPoints.add(in2);
                     toRemove.add(boundaryAlgorithm1(circle, radius,in1, a1, in2, a2, newBS, sp));
                    } else {
                        i += spl;
                        //i = intersectionPoints.size();
                        toRemove.add(boundaryAlgorithm2(circle, radius, in1, a1, in2, a2, intersectionPoints, usedPoints, newBS, sp));
                    }*/
                    //System.out.println("to : " + sp.id);
                    //toRemove.add(boundaryAlgorithm2(circle, radius, in1, a1, in2, a2, ps, usedPoints, newBS, sp));
                    boundaryAlgorithm2(circle, radius, in1, a1, in2, a2, ps, usedPoints, toRemove, newBS, sp, otherPatch);
                    i += ps.size();
                    intersectionPoints.removeAll(ps);
                    //System.out.println("did: " + sp.id);
                    //sp.boundaries.remove(a.bOwner);f
                }
                /*sp.boundaries.removeAll(toRemove);
                sp.boundaries.addAll(newBS);*/
                //if (newBS.size() > 2){
                //    System.out.println("three new boundaries for: " + sp.id);
                //}

                updatePatchBoundaries(sp, toRemove, newBS);
                for (int j = 0; j < newBS.size(); ++j){
                    Boundary b = newBS.get(j);
                    if (b.nestedBoundaries.size() > 0){
                        continue;
                    }
                    boolean valid = false;
                    for (int k = 0; k < b.arcs.size(); ++k){
                        Arc a = b.arcs.get(k);
                        if (a.cuspTriangle != null || a.torus != null){
                            valid = true;
                            break;
                        }
                    }
                    if (!valid){
                        System.out.println("possible invalid boundary for: " + sp.id);
                    }
                }
                boolean check = checkBoundaryOwn(sp);
            }
        } catch (Exception e){
            e.printStackTrace();
            System.err.println("for: " + sp.id);
        }
    }

    private static Map<Integer, List<Plane>> planes = new TreeMap<>();
    private static List<Plane> planePool = new ArrayList<>(50);
    private static boolean planePoolInitialized = false;
    private static int nextPlaneID = 0;
    private static Map<Integer, Map<Integer, List<Point>>> moip = new TreeMap<>();
    //private static Map<Integer, Map<Integer, PatchBridge>> bridgeArcs = new TreeMap<>();
    //private static Map<Integer, List<ArcDivide>> divides = new TreeMap<>();
    private static SphericalPatch curr;
    private static Plane currCirc;
    private static double currRad;
    private static List<Point> currInt;
    private static int currIter;

    private static void exp(){
        SurfaceParser.exportCP(curr, "/home/radoslav/objs/cp_" + curr.id + "_" + currIter + ".obj");
        SurfaceParser.exportCircle(currCirc, currRad, currInt.get(0), "/home/radoslav/objs/cir_" + curr.id + "_" + currIter + ".obj");
        SurfaceParser.exportPoints(currInt, currCirc.v, "/home/radoslav/objs/poi_" + curr.id + "_" + currIter + ".obj");
    }

    private static Boundary _b = new Boundary();
    private static Arc _a1 = new Arc(new Point(0, 0, 0), 1.0);
    private static Arc _a2 = new Arc(new Point(0, 0, 0), 1.0);
    private static List<Point> vrtsPool = new ArrayList<>(17);
    private static List<Neighbor<double[], SphericalPatch>> neighbors = new ArrayList<>(50);
    private static Plane rho = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));
    private static List<Boundary> removeFromSP = new ArrayList<>();
    private static List<Boundary> processed = new ArrayList<>();

    private static void trimConcavePatch(SphericalPatch sp){
        if (!planePoolInitialized){
            for (int i = 0; i < 50; ++i){
                planePool.add(i, new Plane(new Point(0, 0, 0), new Vector(0, 0, 0)));
            }
            planePoolInitialized = true;
        }
        if (vrtsPool.size() == 0){
            for (int i = 0; i < 17; ++i){
                vrtsPool.add(new Point(0, 0, 0));
            }
            _b.arcs.add(_a1);
            _b.arcs.add(_a2);
            //Arc fff = sp.boundaries.get(0).arcs.get(0);
            //Arc _bah = ArcUtil.cloneArc(fff);
            //_bah.vrts.clear();
            //_bah.vrts.add(_bah.end1);
            //_bah.end2 = Point.translatePoint(_bah.center, _bah.toEnd1.multiply(-_bah.radius));
            //_bah.vrts.add(_bah.end2);
            //_bah.setEndPoints(_bah.end1, _bah.end2, false);
            //ArcUtil.refineArc(_bah, SesConfig.edgeLimit, false, 0, false);
            //System.out.println("here she comes, watch out boy, shell chew you up, shes a manEATER");
        }
        try {
            neighbors.clear();
            //List<Neighbor<double[], SphericalPatch>> neighbors = new ArrayList<>();
            Surface.probeTree.range(sp.sphere.center.getData(), 2 * SesConfig.probeRadius, neighbors); //causes slf4j warning
            Collections.sort(neighbors, new Comparator<Neighbor<double[], SphericalPatch>>() {
                @Override
                public int compare(Neighbor<double[], SphericalPatch> o1, Neighbor<double[], SphericalPatch> o2) {
                    if (Math.abs(o1.distance - o2.distance) < 0.001) {
                        return 0;
                    }
                    return (o1.distance - o2.distance > 0.0) ? -1 : 1;
                }
            });
            boolean trimmed = false;
            //int i = 0;
            //planes.put(sp.id, new ArrayList<>());

            nextPlaneID = 0;
            for (int i = 0; i < sp.boundaries.size(); ++i){
                Boundary b = sp.boundaries.get(i);
                for (int j = 0; j < b.arcs.size(); ++j){
                    Arc a = b.arcs.get(j);
                    if (nextPlaneID >= planePool.size()){
                        planePool.add(new Plane(new Point(0, 0, 0), new Vector(0, 0, 0)));
                    }
                    Plane p = planePool.get(nextPlaneID);
                    p.redefine(a.center, a.normal);
                    nextPlaneID++;
                    //Plane p = new Plane(a.center, a.normal);
                    //planes.get(sp.id).add(p);
                }
            }
            currIter = 0;
            curr = sp;
            for (int i = 0; i < neighbors.size(); ++i) {
                Neighbor<double[], SphericalPatch> n = neighbors.get(i);
                if (n.value == sp) {
                    continue;
                }
                SphericalPatch sp2 = n.value;
                if (sp.intersectingPatches.contains(sp2.id)) {
                    continue;
                }
                if (nextPlaneID >= planePool.size()){
                    planePool.add(new Plane(new Point(0, 0, 0), new Vector(0, 0, 0)));
                }
                Plane intersectingPlane = planePool.get(nextPlaneID);
                Point center = intersectingPlane.p;
                double radius = computeIntersectionCircle(sp.sphere.center, sp2.sphere.center, center, SesConfig.probeRadius);
                intersectingPlane.v.changeVector(sp.sphere.center, center).makeUnit();
                intersectingPlane.redefine(intersectingPlane.p, intersectingPlane.v);
                //Vector nV = Point.subtractPoints(sp.sphere.center, center).makeUnit();

                if (Point.distance(sp.sphere.center, sp2.sphere.center) < 0.008){
                    continue;
                }
                //if (nextPlaneID >= planePool.size()){
                //    planePool.add(new Plane(new Point(0, 0, 0), new Vector(0, 0, 0)));
                //}
                //Plane p = planePool.get(nextPlaneID++);
                //p.redefine(center, nV);
                //Plane p = new Plane(center, nV);
                intersectionPoints.clear();
                exclude.clear();
                findIntersectionPoints(sp, center, radius, intersectionPoints, exclude);

                currInt = intersectionPoints;
                currCirc = intersectingPlane;
                currRad = radius;
                toRemove.clear();
                if (sp.id == 30419 && sp2.id == 30435){
                    int cd = 4;
                }
                if (sp.id == 475 && sp2.id == 489) {
                    int cd = 4;
                }
                if (sp.id == 5543){
                    int dfff = 34;
                }
                if (intersectionPoints.size() > 1) {
                    if (!pointsLieOnPatch(sp2, intersectionPoints) && sp.patchNormal.dotProduct(sp2.patchNormal) > 0.8){
                        continue;
                    }
                    boolean identicalIntersector = false;
                    for (int j = 0; j < nextPlaneID; ++j){
                        if (planePool.get(j).isIdenticalWith(intersectingPlane)){
                            identicalIntersector = true;
                            break;
                        }
                    }
                    if (!identicalIntersector){
                        nextPlaneID++;
                    }
                    if (!identicalIntersector){//p!identicalIntersector){//lanes.get(sp.id).stream().noneMatch(plane -> plane.isIdenticalWith(p))) {
                        //planes.get(sp.id).add(p);
                        sp.intersectingPatches.add(sp2.id);

                        generateNewBoundaries2(sp, intersectionPoints, intersectingPlane, radius, sp2.id,false);
                        //i++;
                        currIter++;
//                        if (!sp.trimmed){
//                            sp.trimmed = true;
//                            Surface.trimmedTriangles++;
//                        }
                    }
                } else if (intersectionPoints.size() == 0) {
                    //Boundary newB = ArcUtil.generateCircularBoundary(p, radius);
                    ArcUtil.redefineBoundary(_b, intersectingPlane, radius, vrtsPool, 45);
                    ArcUtil.buildEdges(_b, true);
                    boolean nest = false;
                    //List<Boundary> removeFromSP = new ArrayList<>();
                    //List<Boundary> processed = new ArrayList<>();
                    removeFromSP.clear();
                    processed.clear();
                    Boundary _newB = null;
                    for (int j = 0; j < sp.boundaries.size(); ++j){
                        Boundary b = sp.boundaries.get(j);
                        if (removeFromSP.contains(b) || processed.contains(b)){
                            continue;
                        }
                        boolean isInside = true;
                        for (int k = 0; k < b.arcs.size(); ++k){
                            Arc a = b.arcs.get(k);
                            //Plane rho = new Plane(a.center, a.normal);
                            rho.redefine(a.center, a.normal);
                            //isInside = isInside && _b.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0); //newB -> _b
                            for (int l = 0; l < _b.vrts.size(); ++l){
                                if (!(rho.checkPointLocation(_b.vrts.get(l)) > 0.0)){
                                    isInside = false;
                                    break;
                                }
                            }
                        }
                        if (isInside){
                            for (int k = 0; k < b.nestedBoundaries.size(); ++k){
                                Boundary nb = b.nestedBoundaries.get(k);
                                for (int l = 0; l < nb.arcs.size(); ++l){
                                    Arc a = nb.arcs.get(l);
                                    //Plane rho = new Plane(a.center, a.normal);
                                    rho.redefine(a.center, a.normal);
                                    //isInside = isInside && _b.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
                                    for (int m = 0; m < _b.vrts.size(); ++m){
                                        if (!(rho.checkPointLocation(_b.vrts.get(m)) > 0.0)){
                                            isInside = false;
                                            break;
                                        }
                                    }
                                }
                            }
                            if (isInside) {
                                _newB = new Boundary();
                                Point _center = new Point(_b.arcs.get(0).center);
                                Arc _a1 = ArcUtil.cloneArc(_b.arcs.get(0));
                                //Arc _a2 = ArcUtil.cloneArc(_b.arcs.get(1));
                                _a1.center = _center;
                                Arc _a2 = new Arc(_a1.center, _a1.radius);
                                _a2.setEndPoints(_a1.end2, _a1.end1, false);
                                _a2.setNormal(_a1.normal);
                                _a2.vrts.add(_a2.end1);
                                for (int k = 1; k < _b.arcs.get(1).vrts.size() - 1; ++k){
                                    _a2.vrts.add(new Point(_b.arcs.get(1).vrts.get(k)));
                                }
                                _a2.vrts.add(_a2.end2);
                                _newB.arcs.add(_a1);
                                _newB.arcs.add(_a2);
                                ArcUtil.buildEdges(_newB, true);
                                _newB.nestedBoundaries.add(b);
                                _newB.nestedBoundaries.addAll(b.nestedBoundaries);
                                for (int k = 0; k < _newB.nestedBoundaries.size(); ++k) {
                                    Boundary nb = _newB.nestedBoundaries.get(k);
                                    nb.nestedBoundaries.add(_newB);
                                }
                                toRemove.clear();
                                for (int k = 0; k < _newB.nestedBoundaries.size(); ++k){
                                    Boundary nb = _newB.nestedBoundaries.get(k);
                                    for (int l = 0; l < nb.arcs.size(); ++l){
                                        Arc a = nb.arcs.get(l);
                                        //if (!a.vrts.stream().allMatch(v -> intersectingPlane.checkPointLocation(v) > 0.0)){
                                        //    toRemove.add(nb);
                                        //    break;
                                        //}
                                        for (int m = 0; m < a.vrts.size(); ++m){
                                            if (!(intersectingPlane.checkPointLocation(a.vrts.get(m)) > 0.0)){
                                                toRemove.add(nb);
                                                break;
                                            }
                                        }
                                    }
                                }
                                _newB.nestedBoundaries.removeAll(toRemove);
                                removeFromSP.addAll(toRemove);
                                for (int k = 0; k < _newB.nestedBoundaries.size(); ++k){
                                    Boundary nb = _newB.nestedBoundaries.get(k);
                                    nb.nestedBoundaries.removeAll(toRemove);
                                }
                                processed.addAll(_newB.nestedBoundaries);
                                nest = true;
                            }
                        }
                    }
                    if (nest){
                        _newB.patch = sp;
                        sp.boundaries.removeAll(removeFromSP);
                        sp.boundaries.add(_newB);
                    }

                }
            }
            int a = 8777;
        } catch (Exception e){
            e.printStackTrace();
        }
    }
    private static Vector vInt = new Vector(0, 0, 0);
    private static Point pInt = new Point(0, 0, 0);
    private static Vector hypo = new Vector(0, 0, 0);
    private static Point in1 = new Point(0, 0, 0);
    private static Point in2 = new Point(0, 0, 0);
    private static Point midOfChord = new Point(0, 0, 0);
    private static void findIntersectionPoints(SphericalPatch sp, Point circle, double radius, List<Point> intersectionPoints, List<Arc> exclude){
        //Vector n = Point.subtractPoints(sp.sphere.center, circle).makeUnit();
        //Plane p = new Plane(circle, n);
        circleN.changeVector(sp.sphere.center, circle).makeUnit();
        p1.redefine(circle, circleN);
        for (int i = 0; i < sp.boundaries.size(); ++i){
            Boundary b = sp.boundaries.get(i);
            for (int j = 0; j < b.arcs.size(); ++j){
                Arc a = b.arcs.get(j);
                /*if (a.torus != null || exclude != null && (a == exclude || a == exclude.next || a == exclude.prev)){
                        continue;
                }*/


                //if (a.torus != null || exclude.size() > 0 && (exclude.stream().anyMatch(a_ -> a_ == a || a_.next == a || a_.prev == a))){
                //    continue;
                //}
                if (a.torus != null){
                    continue;
                } else if (exclude.size() > 0){
                    boolean _continue = false;
                    for (int k = 0; k < exclude.size(); ++k){
                        Arc a_ = exclude.get(k);
                        if (a_ == a || a_.next == a || a_.prev == a){
                           _continue = true;
                           break;
                        }
                    }
                    if (_continue){
                        continue;
                    }
                }
                //boolean allInside = a.vrts.stream().allMatch(v -> p.checkPointLocation(v) > 0.0);
                /*boolean allInside = p.checkPointLocation(a.end1) > 0.0 && p.checkPointLocation(a.mid) > 0.0 && p.checkPointLocation(a.end2) > 0.0;
                if (allInside){
                    continue;
                }
                //boolean allOutside = a.vrts.stream().allMatch(v -> p.checkPointLocation(v) < 0.0);
                boolean allOutside = p.checkPointLocation(a.end1) < 0.0 && p.checkPointLocation(a.mid) < 0.0 && p.checkPointLocation(a.end2) < 0.0;
                if (allOutside){
                    continue;
                }*/
                /*if (Math.abs(p.checkPointLocation(a.end1)) < 0.001){
                    if (p.checkPointLocation(a.end2) > 0.0 && p.checkPointLocation(a.prev.end1) > 0.0){
                        continue;
                    }
                } else if (Math.abs(p.checkPointLocation(a.end2)) < 0.001){
                    if (p.checkPointLocation(a.end1) > 0.0 && p.checkPointLocation(a.next.end2) > 0.0){
                        continue;
                    }
                }*/
                //Plane p2 = new Plane(a.center, a.normal);
                p2.redefine(a.center, a.normal);
                //Vector vInt = new Vector(0,0,0);
                //Point pInt = new Point(0,0,0);
                if (Plane.getIntersectionLine(p1, p2, vInt, pInt)){
                    //Vector hypo = Point.subtractPoints(circle, pInt);
                    hypo.changeVector(circle, pInt);
                    double odvesna = vInt.dotProduct(hypo);
                    double dist = Math.sqrt(hypo.dotProduct(hypo) - odvesna * odvesna);
                    if (dist - radius < 0.0){
                        //Point midTetiva = Point.translatePoint(pInt, Vector.scaleVector(vInt, odvesna));
                        midOfChord.assignTranslation(pInt, vInt.multiply(odvesna));
                        double odv2 = Math.sqrt(radius * radius - dist * dist);
                        //Point in1 = Point.translatePoint(midTetiva, Vector.scaleVector(vInt, odv2));
                        //Point in2 = Point.translatePoint(midTetiva, Vector.scaleVector(vInt, -odv2));
                        in1.assignTranslation(midOfChord, vInt.makeUnit().multiply(odv2));
                        in2.assignTranslation(midOfChord, vInt.multiply(-1.0));
                        if (a.isInside(in1)){// && ArcUtil.findContainingArc(in1, p1, sp, null) != null){
                            boolean foundSimilar = false;
                            for (int k = 0; k < intersectionPoints.size(); ++k){
                                if (Point.distance(in1, intersectionPoints.get(k)) < 0.01){
                                   foundSimilar = true;
                                   break;
                                }
                            }
                            if (!foundSimilar){//intersectionPoints.stream().noneMatch(v -> Point.distance(in1, v) < 0.01)){
                                Point _p = new Point(in1);
                                _p.arc = a;
                                if (Point.distance(a.end1, _p) < 0.001 && nextSign(_p, a, p1) < 0.0){
                                    _p.arc = a.prev;
                                } else if (Point.distance(a.end2, _p) < 0.001 && nextSign(_p, a, p1) > 0.0){
                                    _p.arc = a.next;
                                }
                                intersectionPoints.add(_p);
                                //if (a.cuspTriangle != null){
                                //    if (!a.vrts.stream().allMatch(point -> p1.checkPointLocation(point) > 0.0 || p1.distanceFromPlane(point) < 0.002)){
                                //        intersectionPoints.remove(_p);
                                //    }
                                //}
                                if (a.cuspTriangle != null){
                                    for (int k = 0; k < a.vrts.size(); ++k){
                                        Point point = a.vrts.get(k);
                                        if (!(p1.checkPointLocation(point) > 0.0 || p1.distanceFromPlane(point) < 0.002)){
                                            intersectionPoints.remove(_p);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        if (a.isInside(in2)){// && ArcUtil.findContainingArc(in2, p1, sp, null) != null) {
                            boolean foundSimilar = false;
                            for (int k = 0; k < intersectionPoints.size(); ++k){
                                if (Point.distance(in2, intersectionPoints.get(k)) < 0.01){
                                    foundSimilar = true;
                                    break;
                                }
                            }
                            if (!foundSimilar){//*/intersectionPoints.stream().noneMatch(v -> Point.distance(in2, v) < 0.01)){
                                Point _p = new Point(in2);
                                _p.arc = a;
                                if (Point.distance(a.end1, _p) < 0.001 && nextSign(_p, a, p1) < 0.0){
                                    _p.arc = a.prev;
                                } else if (Point.distance(a.end2, _p) < 0.001 && nextSign(_p, a, p1) > 0.0){
                                    _p.arc = a.next;
                                }
                                intersectionPoints.add(_p);
                                //if (a.cuspTriangle != null){
                                //    if (!a.vrts.stream().allMatch(point -> p1.checkPointLocation(point) > 0.0 || p1.distanceFromPlane(point) < 0.002)){
                                //        intersectionPoints.remove(_p);
                                //    }
                                //}
                                if (a.cuspTriangle != null){
                                    for (int k = 0; k < a.vrts.size(); ++k){
                                        Point point = a.vrts.get(k);
                                        if (!(p1.checkPointLocation(point) > 0.0 || p1.distanceFromPlane(point) < 0.002)){
                                            intersectionPoints.remove(_p);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    private static Vector n = new Vector(0, 0, 0);


    private static Point findOptimalPoint(List<Point> points, List<Point> usedPoints, SphericalPatch sp, Plane plane){
        try {
            for (int i = 0; i < points.size(); ++i) {
                Point p = points.get(i);
                if (usedPoints.contains(p)){
                    continue;
                }
                Arc a = p.arc;//ArcUtil.findContainingArc(p, plane, sp, null);
                if (a == null){
                    int r = 42;
                }
                if (isOptimal(p, a, plane)){
                    return p;
                }
            }
        } catch (Exception e){
            e.printStackTrace();
        }
        return null;
    }


    private static List<Point> intersectionPoints = new ArrayList<>();
    private static List<Arc> exclude = new ArrayList<>();
    private static List<Point> invalid = new ArrayList<>();
    public static void trimSelfIntersectingPatch(Arc arc){
        try {
            if (!arc.valid || !arc.intersecting){
                return;
            }
            SphericalPatch sp = arc.owner;
            if (sp.id == 30419){
                int daf = 3;
            }
            //List<Point> intersectionPoints = new ArrayList<>();
            intersectionPoints.clear();
            //List<Arc> exclude = new ArrayList<>();
            exclude.clear();
            exclude.add(arc);
            invalid.clear();
            findIntersectionPoints(sp, arc.center, arc.radius, intersectionPoints, exclude);
            //List<Point> invalid = new ArrayList<>();
            for (int i = 0; i < intersectionPoints.size(); ++i){
                Point v = intersectionPoints.get(i);
                if (Point.distance(v, arc.prev.end1) < 0.0015 || Point.distance(v, arc.next.end2) < 0.0015){
                    invalid.add(v);
                }
            }
            intersectionPoints.removeAll(invalid);
            if (intersectionPoints.size() > 1){ //theoretically at most 2 intersection points should be encountered(and in most cases), 1 point is special case which is not handled right now(it is rare)
                for (int i = 0; i < sp.boundaries.size(); ++i){
                    Boundary b = sp.boundaries.get(i);
                    for (int j = 0; j < b.arcs.size(); ++j){
                        Arc a = b.arcs.get(j);
                        a.valid = false;
                    }
                }
                Plane circle = rho;// new Plane(arc.center, arc.normal);
                rho.redefine(arc.center, arc.normal);
                //List<Boundary> newBS = new ArrayList<>();
                newBS.clear();
                Point first = ArcUtil.findClosestPointOnCircle(intersectionPoints, arc.end1, true, arc.center, arc.normal, true);
                intersectionPoints.remove(first);
                Arc a = first.arc;//ArcUtil.findContainingArc(first, circle, sp, arc);
                if (Point.distance(a.end1, first) < 0.001 && nextSign(first, a, circle) < 0.0){
                    a = a.prev;
                } else if (Point.distance(a.end2, first) < 0.001 && nextSign(first, a, circle) > 0.0){
                    a = a.next;
                }
                if (a == null){
                    int c = 4;
                }
                Boundary b = new Boundary();
                Arc newA = (Point.distance(first, arc.end1) < 0.003) ? null : new Arc(arc.center, arc.radius);
                if (newA != null) {
                    newA.vrts.add(arc.end1);
                    newA.vrts.add(first);
                    newA.setEndPoints(arc.end1, first, false);
                    newA.setNormal(arc.normal);
                    if (arc.cuspTriangle != null){
                        int subdLevel = ArcUtil.getSubdivisionLevel(arc);
                        newA.cuspTriangle = arc.cuspTriangle;
                        if (newA.cuspTriangle.left == arc){
                            newA.cuspTriangle.left = newA;
                        } else {
                            newA.cuspTriangle.right = newA;
                        }
                        ArcUtil.refineArc(newA, 0, true, subdLevel, false);
                    } else {
                        ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                    }
                    if (arc.torus != null){
                        if (arc.torus.tr1 != null){
                            if (arc == arc.torus.tr1.left){
                                arc.torus.tr1.left = newA;
                            } else if (arc == arc.torus.tr1.right){
                                arc.torus.tr1.right = newA;
                            } else if (arc == arc.torus.tr2.left){
                                arc.torus.tr2.left = newA;
                            } else if (arc == arc.torus.tr2.right){
                                arc.torus.tr2.right = newA;
                            }
                        } else {
                            if (arc.torus.concavePatchArcs.get(0) == arc){
                                arc.torus.concavePatchArcs.remove(arc);
                                arc.torus.concavePatchArcs.add(newA);
                            } else {
                                arc.torus.concavePatchArcs.remove(arc);
                                arc.torus.concavePatchArcs.add(newA);
                            }
                        }
                        newA.torus = arc.torus;
                    }
                } else {
                    first = arc.end1;
                }
                /*newA.prev = arc.prev;
                newA.prev.next = newA;*/

                Arc newA2 = new Arc(a.center, a.radius);
                newA2.vrts.add(first);
                newA2.vrts.add(a.end2);
                newA2.setEndPoints(first, a.end2, false);
                newA2.setNormal(a.normal);
                if (a.cuspTriangle != null){
                    int subdLevel = ArcUtil.getSubdivisionLevel(a);
                    newA2.cuspTriangle = a.cuspTriangle;
                    if (a.cuspTriangle.left == a){
                        newA2.cuspTriangle.left = newA2;
                    } else {
                        newA2.cuspTriangle.right = newA2;
                    }
                    ArcUtil.refineArc(newA2, 0, true, subdLevel, false);
                } else {
                    ArcUtil.refineArc(newA2, Surface.maxEdgeLen, false, 0, false);
                }
                if (a.torus != null){
                    if (a.torus.tr1 != null){
                        if (a == a.torus.tr1.left){
                            a.torus.tr1.left = newA2;
                        } else if (a == a.torus.tr1.right){
                            a.torus.tr1.right = newA2;
                        } else if (a == a.torus.tr2.left){
                            a.torus.tr2.left = newA2;
                        } else if (a == a.torus.tr2.right){
                            a.torus.tr2.right = newA2;
                        }
                    } else {
                        if (a.torus.concavePatchArcs.get(0) == a){
                            a.torus.concavePatchArcs.remove(a);
                            a.torus.concavePatchArcs.add(newA2);
                        } else {
                            a.torus.concavePatchArcs.remove(a);
                            a.torus.concavePatchArcs.add(newA2);
                        }
                    }
                    newA2.torus = a.torus;
                }
                /*newA2.prev = newA;
                newA.next = newA2;
                newA2.next = a.next;
                newA2.next.prev = newA2;*/

                if (newA != null) {
                    b.arcs.add(newA);
                }
                b.arcs.add(newA2);
                Arc start = arc;
                Arc a_ = a.next;
                while (a_ != start) {
                    b.arcs.add(a_);
                    if (a_ == null){
                        System.out.println(" ");
                    }
                    if (a_.next == null){
                        System.out.println(" ");
                    }
                    a_ = a_.next;
                }

                b.patch = sp;
                ArcUtil.buildEdges(b, true);

                newBS.add(b);


                Point second = intersectionPoints.get(0); //remaining intersection point
                a = second.arc;//ArcUtil.findContainingArc(second, circle, sp, arc);
                if (Point.distance(second, a.end1) < 0.001 && nextSign(second, a, circle) < 0.0){
                    a = a.prev;
                } else if (Point.distance(second, a.end2) < 0.001 && nextSign(second, a, circle) > 0.0){
                    a = a.next;
                }
                b = new Boundary();
                newA = new Arc(a.center, a.radius);
                newA.vrts.add(a.end1);
                newA.vrts.add(second);
                newA.setEndPoints(a.end1, second, false);
                newA.setNormal(a.normal);
                if (a.cuspTriangle != null){
                    int subdLevel = ArcUtil.getSubdivisionLevel(a);
                    newA.cuspTriangle = a.cuspTriangle;
                    if (a.cuspTriangle.left == a){
                        newA.cuspTriangle.left = newA;
                    } else {
                        newA.cuspTriangle.right = newA;
                    }
                    ArcUtil.refineArc(newA, 0, true, subdLevel, false);
                } else {
                    ArcUtil.refineArc(newA, Surface.maxEdgeLen, false, 0, false);
                }

                if (a.torus != null){
                    if (a.torus.tr1 != null){
                        if (a == a.torus.tr1.left){
                            a.torus.tr1.left = newA;
                        } else if (a == a.torus.tr1.right){
                            a.torus.tr1.right = newA;
                        } else if (a == a.torus.tr2.left){
                            a.torus.tr2.left = newA;
                        } else if (a == a.torus.tr2.right){
                            a.torus.tr2.right = newA;
                        }
                    } else {
                        if (a.torus.concavePatchArcs.get(0) == a){
                            a.torus.concavePatchArcs.remove(a);
                            a.torus.concavePatchArcs.add(newA);
                        } else {
                            a.torus.concavePatchArcs.remove(a);
                            a.torus.concavePatchArcs.add(newA);
                        }
                    }
                    newA.torus = a.torus;
                }
                /*newA.prev = a.prev;
                newA.prev.next = newA;*/

                newA2 = (Point.distance(second, arc.end2) < 0.003) ? null : new Arc(arc.center, arc.radius);
                if (newA2 != null) {
                    newA2.vrts.add(second);
                    newA2.vrts.add(arc.end2);
                    newA2.setEndPoints(second, arc.end2, false);
                    newA2.setNormal(arc.normal);
                    if (arc.cuspTriangle != null){
                        int subdLevel = ArcUtil.getSubdivisionLevel(arc);
                        newA2.cuspTriangle = arc.cuspTriangle;
                        if (arc.cuspTriangle.left == arc){
                            newA2.cuspTriangle.left = newA2;
                        } else {
                            newA2.cuspTriangle.right = newA2;
                        }
                        ArcUtil.refineArc(newA2, 0, true, subdLevel, false);
                    } else {
                        ArcUtil.refineArc(newA2, Surface.maxEdgeLen, false, 0, false);
                    }
                    if (arc.torus != null){
                        if (arc.torus.tr1 != null){
                            if (arc == arc.torus.tr1.left){
                                arc.torus.tr1.left = newA2;
                            } else if (arc == arc.torus.tr1.right){
                                arc.torus.tr1.right = newA2;
                            } else if (arc == arc.torus.tr2.left){
                                arc.torus.tr2.left = newA2;
                            } else if (arc == arc.torus.tr2.right){
                                arc.torus.tr2.right = newA2;
                            }
                        } else {
                            if (arc.torus.concavePatchArcs.get(0) == arc){
                                arc.torus.concavePatchArcs.remove(arc);
                                arc.torus.concavePatchArcs.add(newA2);
                            } else {
                                arc.torus.concavePatchArcs.remove(arc);
                                arc.torus.concavePatchArcs.add(newA2);
                            }
                        }
                        newA2.torus = arc.torus;
                    }
                } else {
                    second = arc.end2;
                    newA.end2 = second;
                    newA.vrts.remove(newA.vrts.size() - 1);
                    newA.vrts.add(second);
                }
                /*newA.next = newA2;
                /newA.next.prev = newA;
                newA2.next = arc.next;
                newA2.next.prev = newA2;*/
                start = a;
                a_ = arc.next;
                b.arcs.add(newA);
                if (newA2 != null) {
                    b.arcs.add(newA2);
                }
                while (a_ != start) {
                    b.arcs.add(a_);
                    a_ = a_.next;
                }
                b.patch = sp;
                ArcUtil.buildEdges(b, true);
                newBS.add(b);
                sp.boundaries.clear();
                sp.boundaries.addAll(newBS);
            } else if (intersectionPoints.size() == 1){
                System.out.println("ONLY ONE INT POINT");
            }
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    private static int shortArcs(SphericalPatch sp){
        int num = 0;
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (Point.distance(a.end1, a.end2) < 0.0015){
                    num++;
                }
            }
        }
        return num;
    }

    private static int shortArcs(Boundary b){
        int num = 0;
        for (Arc a : b.arcs){
            if (Point.distance(a.end1, a.end2) < 0.0015){
                num++;
            }
        }
        return num;
    }


    private static List<Point> usedPoints = new ArrayList<>();
    private static List<Point> usablePoints = new ArrayList<>();
    private static List<Point> usedPoints2 = new ArrayList<>();
    private static List<Point> getUsablePoints(List<Point> intersectionPoints, Point origin, Point start, SphericalPatch sp, Plane circle){
        //List<Point> usedPoints = new ArrayList<>();
        //List<Point> usablePoints = new ArrayList<>(intersectionPoints);
        usedPoints2.clear();
        usablePoints.clear();
        usablePoints.addAll(intersectionPoints);
        Arc originArc = origin.arc;//ArcUtil.findContainingArc(origin, circle, sp, null);
        Arc startArc = start.arc;//ArcUtil.findContainingArc(start, circle, sp, null);
        Arc endArc = null;
        Arc a = null;
        Point end = null;
        boolean init = true;
        boolean bridge = (circle.checkPointLocation(startArc.end1) > 0.0);
        do {
            if (init){
                usedPoints2.add(start);
                usablePoints.remove(start);
                /*if (start == origin){
                    return usedPoints;
                }*/
                startArc = start.arc;//ArcUtil.findContainingArc(start, circle, sp, null);
                end = ArcUtil.findClosestPointOnCircle(intersectionPoints, start, false, circle.p, circle.v, true);
                endArc = end.arc;//ArcUtil.findContainingArc(end, circle, sp, null);
                a = (startArc == endArc) ? ((ArcUtil.getOrder(startArc, start, end) < 0) ? startArc : startArc.next) : startArc.next;
                if (startArc.bOwner != endArc.bOwner && startArc.bOwner.nestedBoundaries.contains(endArc.bOwner)){
                    a = endArc;
                }
                if (end == origin){
                    a = endArc;
                }
                if (isOptimal(start, startArc, circle)){
                    a = endArc;
                }
                /*if (bridge){
                    a = endArc;
                    bridge = false;
                } else {


                    bridge = true;
                }*/
                init = false;
            } else if (a == endArc){
                start = end;
                init = true;
            } else {
                Point newStart = null;
                for (int i = 0; i < usablePoints.size(); ++i){
                    Point p = usablePoints.get(i);
                    if (a.isInside(p)){
                        //final Arc arc = a;
                        //Optional<Point> otherP = usablePoints.stream().filter(v -> v != p && arc.isInside(v)).findFirst();
                        //newStart = (otherP.isPresent()) ? ((ArcUtil.getOrder(a, p, otherP.get()) < 0) ? p : otherP.get()) : p;
                        Point otherP = null;
                        for (int l = 0; l < usablePoints.size(); ++l){
                            Point v = usablePoints.get(l);
                            if (v != p && a.isInside(v)){
                                otherP = v;
                                break;
                            }
                        }
                        newStart = (otherP != null) ? ((ArcUtil.getOrder(a, p, otherP) < 0) ? p : otherP) : p;
                        break;
                    }
                }
                if (newStart != null){
                    start = newStart;
                    init = true;
                }
                a = a.next;
            }
        } while (init || start != origin);
        return usedPoints2;
    }

    private static boolean checkBoundaryOwn(SphericalPatch sp){
        for (int i = 0; i < sp.boundaries.size(); ++i){
            Boundary b = sp.boundaries.get(i);
            for (int j = 0; j < b.arcs.size(); ++j){
                Arc a = b.arcs.get(j);
                if (a.bOwner != b){
                    return true;
                }
            }
        }
        return false;
    }

    private static boolean checkBoundary(Boundary b){
        for (Arc a : b.arcs){
            if (a.bOwner != b){
                return true;
            }
        }
        return false;
    }

    private static void updatePatchBoundaries(SphericalPatch sp, List<Boundary> toRemove, List<Boundary> toAdd){
        for (int i = 0; i < toRemove.size(); ++i){
            Boundary b = toRemove.get(i);
            for (int j = 0; j < b.nestedBoundaries.size(); ++j){
                Boundary nb = b.nestedBoundaries.get(j);
                nb.nestedBoundaries.remove(b);
            }
        }
        for (int i = 0; i < toRemove.size(); ++i){
            Boundary b = toRemove.get(i);
            if (b.mergeSplit.size() == 1){
                Boundary b2 = b.mergeSplit.get(0);
                if (b2.mergeSplit.size() == 1){
                    for (int j = 0; j < b.nestedBoundaries.size(); ++j){
                        Boundary nb = b.nestedBoundaries.get(j);
                        if (areNested(b2, nb)){
                            b2.nestedBoundaries.add(nb);
                            nb.nestedBoundaries.add(b2);
                        } else {
                            sp.boundaries.remove(nb);
                            for (int k = 0; k < nb.nestedBoundaries.size(); ++k){
                                Boundary nb_ = nb.nestedBoundaries.get(k);
                                nb_.nestedBoundaries.remove(nb);
                            }
                        }
                    }
                } else if (b2.mergeSplit.size() > 1){
                    for (int j = 0; j < b.nestedBoundaries.size(); ++j){
                        Boundary nb = b.nestedBoundaries.get(j);
                        if (areNested(b2, nb)){
                            b2.nestedBoundaries.add(nb);
                            nb.nestedBoundaries.add(b2);
                        } else {
                            sp.boundaries.remove(nb);
                            for (int k = 0; k < nb.nestedBoundaries.size(); ++k){
                                Boundary nb_ = nb.nestedBoundaries.get(k);
                                nb_.nestedBoundaries.remove(nb);
                            }
                        }
                    }

                }
                //b2.mergeSplit.stream().forEach(b_ -> b_.mergeSplit.clear());
                for (int j = 0; j < b2.mergeSplit.size(); ++j){
                    b2.mergeSplit.get(j).mergeSplit.clear();
                }
            } else if (b.mergeSplit.size() > 1){
                for (int j = 0; j < b.mergeSplit.size(); ++j){
                    Boundary newB = b.mergeSplit.get(j);
                    for (int k = 0; k < b.nestedBoundaries.size(); ++k){
                        Boundary nb = b.nestedBoundaries.get(k);
                        if (areNested(newB, nb)){
                            newB.nestedBoundaries.add(nb);
                            nb.nestedBoundaries.add(newB);
                        } else {
                            sp.boundaries.remove(nb);
                            for (int l = 0; l < nb.nestedBoundaries.size(); ++l){
                                Boundary nb_ = nb.nestedBoundaries.get(l);
                                nb_.nestedBoundaries.remove(nb);
                            }
                        }
                    }
                    newB.mergeSplit.clear();
                }
                b.mergeSplit.clear();
            }
        }
        //toAdd.stream().forEach(b -> b.mergeSplit.clear());
        for (int i = 0; i < toAdd.size(); ++i){
            toAdd.get(i).mergeSplit.clear();
        }
        sp.boundaries.removeAll(toRemove);
        sp.boundaries.addAll(toAdd);
    }

    private static boolean areNested(Boundary b1, Boundary b2){
        //boolean nest = b1.arcs.stream().allMatch(a -> {
        //    //Plane rho = new Plane(a.center, a.normal);
        //    rho.redefine(a.center, a.normal);
        //    return b2.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
        //});
        boolean _nest = true;
        for (int i = 0; i < b1.arcs.size(); ++i){
            Arc a = b1.arcs.get(i);
            rho.redefine(a.center, a.normal);
            for (int j = 0; j < b2.vrts.size(); ++j){
                if (!(rho.checkPointLocation(b2.vrts.get(j)) > 0.0)){
                    _nest = false;
                    break;
                }
            }
            if (!_nest){
                break;
            }
        }
        if (_nest){
            //nest = b2.arcs.stream().allMatch(a -> {
            //    //Plane rho = new Plane(a.center, a.normal);
            //    rho.redefine(a.center, a.normal);
            //    return b1.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
            //});
            for (int i = 0; i < b2.arcs.size(); ++i){
                Arc a = b2.arcs.get(i);
                rho.redefine(a.center, a.normal);
                for (int j = 0; j < b1.vrts.size(); ++j){
                    if (!(rho.checkPointLocation(b1.vrts.get(j)) > 0.0)){
                        _nest = false;
                        break;
                    }
                }
                if (!_nest){
                    break;
                }
            }
            if (!_nest){
                return false;
            }
        } else {
            return false;
        }
        return true;
    }

//    public static void addFaceToEdgeFacesMap(SphericalPatch sp, Face f){
//        int sID = (f.a > f.b) ? f.b : f.a;
//        int bID = (sID == f.a) ? f.b : f.a;
//
//        Map<Integer, Map<Integer, List<Face>>> map1 = sp.edgeFacesMap;
//        if (!map1.containsKey(sID)){
//            map1.put(sID, new TreeMap<>());
//        }
//        Map<Integer, List<Face>> map2 = map1.get(sID);
//        if (!map2.containsKey(bID)){
//            map2.put(bID, new ArrayList<>(2));
//        }
//        map2.get(bID).add(f);
//
//        sID = (f.c > f.b) ? f.b : f.c;
//        bID = (sID == f.c) ? f.b : f.c;
//
//        if (!map1.containsKey(sID)){
//            map1.put(sID, new TreeMap<>());
//        }
//        map2 = map1.get(sID);
//        if (!map2.containsKey(bID)){
//            map2.put(bID, new ArrayList<>(2));
//        }
//        map2.get(bID).add(f);
//
//        sID = (f.c > f.a) ? f.a : f.c;
//        bID = (sID == f.c) ? f.a : f.c;
//
//        if (!map1.containsKey(sID)){
//            map1.put(sID, new TreeMap<>());
//        }
//        map2 = map1.get(sID);
//        if (!map2.containsKey(bID)){
//            map2.put(bID, new ArrayList<>(2));
//        }
//        map2.get(bID).add(f);
//    }

//    public static void removeFaceFromEdgeFacesMap(SphericalPatch sp, Face f){
//        Map<Integer, Map<Integer, List<Face>>> map1 = sp.edgeFacesMap;
//        int sID = (f.a > f.b) ? f.b : f.a;
//        int bID = (f.a > f.b) ? f.a : f.b;
//
//        map1.get(sID).get(bID).remove(f);
//
//        sID = (f.b > f.c) ? f.c : f.b;
//        bID = (f.b > f.c) ? f.b : f.c;
//
//        map1.get(sID).get(bID).remove(f);
//
//        sID = (f.a > f.c) ? f.c : f.a;
//        bID = (f.a > f.c) ? f.a : f.c;
//
//        map1.get(sID).get(bID).remove(f);
//    }

//    public static List<Face> retrieveFacesFromEdgeFacesMap(SphericalPatch sp, int a, int b){
//        int small = Math.min(a, b);
//        int big = Math.max(a, b);
//        return sp.edgeFacesMap.get(small).get(big);
//    }

//    public static Face retrieveFirstFaceFromEdgeFacesMap(SphericalPatch sp, int a, int b){
//        List<Face> _f = retrieveFacesFromEdgeFacesMap(sp, a, b);
//        return (_f.size() > 0) ? _f.get(0) : null;
//    }

    public static Vector computeTriangleNormal(Point a, Point b, Point c){
        //return in.assignNormalVectorOf(v1.changeVector(b, a).makeUnit(), v2.changeVector(c, a).makeUnit()).makeUnit();
        return Vector.getNormalVector(Point.subtractPoints(b, a).makeUnit(), Point.subtractPoints(c, a).makeUnit()).makeUnit();
    }

    public static Vector computeTriangleNormal(Point a, Point b, Point c, Vector in){
        in.assignNormalVectorOf(v1.changeVector(b, a).makeUnit(), v2.changeVector(c, a).makeUnit()).makeUnit();
        return in;
    }

    private static boolean pointsLieOnPatch(SphericalPatch sp, List<Point> points){
        for (int i = 0; i < sp.boundaries.size(); ++i){
            Boundary b = sp.boundaries.get(i);
            for (int j = 0; j < b.arcs.size(); ++j){
                Arc a = b.arcs.get(j);
                for (int k = 0; k < points.size(); ++k){
                    Point p = points.get(k);
                    if (a.isInside(p)){
                        return true;
                    }
                }
            }
        }
        return false;
    }

//    private static void trimConcavePatch(SphericalPatch sp, SphericalPatch sp2, boolean force){
//        Point center = new Point(0, 0, 0);
//        double radius = computeIntersectionCircle(sp.sphere.center, sp2.sphere.center, center, SesConfig.probeRadius);
//        Vector nV = Point.subtractPoints(sp.sphere.center, center).makeUnit();
//        Plane p = new Plane(center, nV);
//        List<Point> intersectionPoints = new ArrayList<>();
//        List<Arc> excludeArcs = new ArrayList<>();
//        if (divides.containsKey(sp.id)){
//            divides.get(sp.id).stream().forEach(arcDivide -> {
//                intersectionPoints.add(arcDivide.intersection);
//                excludeArcs.add(arcDivide.newArc);
//            });
//        }
//        findIntersectionPoints(sp, center, radius, intersectionPoints, excludeArcs);
//        if (intersectionPoints.size() > 1){
//            sp.intersectingPatches.add(sp2.id);
//            generateNewBoundaries2(sp, intersectionPoints, p, radius, sp2.id, force);
//        }
//    }

//    private static void putNewBridgeArc(int sp1, int sp2, Arc a){
//        int small = Math.min(sp1, sp2);
//        int big = Math.max(sp1, sp2);
//        if (!bridgeArcs.containsKey(small)){
//            bridgeArcs.put(small, new TreeMap<>());
//        }
//        Map<Integer, PatchBridge> map = bridgeArcs.get(small);
//        if (!map.containsKey(big)){
//            map.put(big, new PatchBridge());
//        }
//        PatchBridge pb = map.get(big);
//        pb.intersections.add(a.end1);
//        pb.intersections.add(a.end2);
//        pb.bridges.add(a);
//    }

//    private static Arc retrieveBridgeArc(int sp1, int sp2, Point in1, Point in2){
//        int small = Math.min(sp1, sp2);
//        int big = Math.max(sp1, sp2);
//        if (!bridgeArcs.containsKey(small)){
//            return null;
//        }
//        Map<Integer, PatchBridge> map = bridgeArcs.get(small);
//        if (!map.containsKey(big)){
//            return null;
//        }
//        PatchBridge pb = map.get(big);
//        Optional<Arc> arc = pb.bridges.stream().filter(a -> Point.distance(a.end1, in1) < 0.001 && Point.distance(a.end2, in2) < 0.001).findFirst();
//        if (arc.isPresent()){
//            return arc.get();
//        }
//        return null;
//    }

    private static boolean _check(SphericalPatch sp){
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (a.cuspTriangle != null){
                    if (a.opposite.owner.id == sp.id){
                        int g = 3;
                        return true;
                    }
                }
            }
        }
        return false;
    }
    private static Point _nextPoint = new Point(0, 0, 0);
    private static Vector _genVector = new Vector(0, 0, 0);
    private static Quaternion _genQuaternion = new Quaternion();
    private static float[] _floatVector = new float[3];
    public static Point genP(Arc a, Point p, int dir){
        //Vector v = Point.subtractPoints(p, a.center).makeUnit();
        _genVector.changeVector(p, a.center).makeUnit();
        //Quaternion q = new Quaternion();
        _genQuaternion.setFromAngleNormalAxis((float)(dir * Math.toRadians(2)), a.normal.getFloatData());
        //float[] _f = new float[3];
        _genQuaternion.rotateVector(_floatVector, 0, _genVector.getFloatData(), 0);
        //Vector u = new Vector(_floatVector);
        //u.makeUnit().multiply(a.radius);
        _genVector.changeVector(_floatVector[0], _floatVector[1], _floatVector[2]);
        _genVector.makeUnit().multiply(a.radius);
        //return Point.translatePoint(a.center, u);
        return _nextPoint.assignTranslation(a.center, _genVector);
    }

    private static boolean isOptimal(Point p, Arc a, Plane circle){
        int dir = 1;
        if (Point.distance(p, a.end2) < 0.001){
            dir = -1;
        }
        Point _p = genP(a, p, dir);
        //Vector ak = Point.subtractPoints(_p, circle.p).multiply(10.f); _genVector.changeVector(_nextPoint, circle.p).multiply(10.f);
        //Point __p = Point.translatePoint(circle.p, ak);
        _genVector.changeVector(_nextPoint, circle.p).multiply(10.f);
        _nextPoint.assignTranslation(circle.p, _genVector);
        return dir * circle.checkPointLocation(_nextPoint) < 0.0;
    }

    public static double nextSign(Point p, Arc a, Plane circle){
        Point _p = genP(a, p, 1);
        //Vector ak = Point.subtractPoints(_p, circle.p).multiply(10.f);
        //Point __p = Point.translatePoint(circle.p, ak);
        _genVector.changeVector(_p, circle.p).multiply(10.f);
        _nextPoint.assignTranslation(circle.p, _genVector);
        return circle.checkPointLocation(_nextPoint);
    }

    private static boolean arcInsideHyperspace(Arc a, Plane circle, Point p){
        boolean inside = true;
        for (int i = 0; i < a.vrts.size(); ++i){
            Point point = a.vrts.get(i);
            if (!(Point.distance(p, point) < 0.001 || circle.checkPointLocation(point) > 0.0 || circle.distanceFromPlane(point) < 0.001)){
               inside = false;
               break;
            }
        }
        return inside;
    }

    private static void refineArcForNvertices(Arc a, int n){
        if (a.vrts.size() >= n){
            return;
        }
        int currSubd = ArcUtil.getSubdivisionLevel(a);
        int target = (int)(Math.log10(n) / Math.log10(2));
        ArcUtil.refineArc(a, SesConfig.edgeLimit, true, (target - currSubd), false);
    }

    private static void equalizeVertexCount(CuspTriangle tr1, CuspTriangle tr2){
        int maxCount = Math.max(Math.max(tr1.left.vrts.size(), tr1.right.vrts.size()), Math.max(tr2.left.vrts.size(), tr2.right.vrts.size()));
        refineArcForNvertices(tr1.left, maxCount);
        refineArcForNvertices(tr1.right, maxCount);
        refineArcForNvertices(tr2.left, maxCount);
        refineArcForNvertices(tr2.right, maxCount);
    }

    public static int getTorusProbeIdx(ToroidalPatch tp, int vertexID){
        if (tp.tr1 != null){
           int firstPatchVertexCount = tp.tr1.base.vrts.size() * tp.tr1.left.vrts.size();
           int arcLen = (vertexID >= firstPatchVertexCount) ? tp.tr2.left.vrts.size() : tp.tr1.left.vrts.size();
           int probeOffset = (vertexID >= firstPatchVertexCount) ? tp.tr1.base.vrts.size() : 0;
           int vertexOffset = (probeOffset > 0) ? firstPatchVertexCount : 0;
           return (vertexID - vertexOffset) / arcLen + probeOffset;
        } else {
           if (tp.concavePatchArcs.size() == 0){
              if (getProbeAxisDistance(tp.probe1, tp.convexPatchArcs.get(0).center, tp.convexPatchArcs.get(1).center) - SesConfig.probeRadius < 0.0){
                  int arcLen = tp.vertices.size() / (2 * tp.convexPatchArcs.get(0).vrts.size());
                  return vertexID / arcLen;
              } else {
                 int arcLen = tp.vertices.size() / tp.convexPatchArcs.get(0).vrts.size();
                 return vertexID / arcLen;
              }
           } else {
               return vertexID / tp.concavePatchArcs.get(0).vrts.size();
           }
        }
    }
}
