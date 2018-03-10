package cz.fi.muni.xmraz3.utils;

import cz.fi.muni.xmraz3.*;
import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Sphere;
import cz.fi.muni.xmraz3.math.Vector;
import smile.neighbor.Neighbor;

import java.util.*;

public class PatchUtil {

    public static double getProbeAxisDistance(Point probe, Point a1, Point a2){
        Vector axis = Point.subtractPoints(a2, a1).makeUnit();
        Vector toProbe = Point.subtractPoints(probe, a1);
        axis.multiply(toProbe.dotProduct(axis));
        return Vector.addVectors(toProbe, axis.multiply(-1)).sqrtMagnitude();
    }

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
                Main.rectangles.remove(tp);
                //System.out.println(" errorrrr");
                return;
            }
            //System.err.println(rp.cxpl1.atom.id + " " + rp.cxpl2.atom.id);
            Arc leftL = tp.concavePatchArcs.get(0);
            Arc rightL = tp.concavePatchArcs.get(1);
            //Arc_ bottom = (leftL.end2 == tp.convexPatchArcs.get(0).end2) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
            Arc bottom = (Point.distance(leftL.end2, tp.convexPatchArcs.get(0).end2) < 0.001) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
            Arc top = (tp.convexPatchArcs.get(0) == bottom) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);

            if (leftL.owner.intersectingPatches.contains(rightL.owner.id)){
                Arc cpl1 = leftL.owner.boundaries.get(0).arcs.stream().filter(a -> a.intersecting).findFirst().get();
                Arc cpl2 = rightL.owner.boundaries.get(0).arcs.stream().filter(a -> a.intersecting).findFirst().get();
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
                for (Boundary b : left.boundaries){
                    for (Arc a : b.arcs){
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
                for (Boundary b : right.boundaries){
                    for (Arc a : b.arcs){
                        if (Point.distance(tr1.base.end1, a.end1) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr1.right = target;
                target = null;
                for (Boundary b : right.boundaries){
                    for (Arc a : b.arcs){
                        if (Point.distance(tr2.base.end2, a.end2) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr2.left = target;
                tr2.cuspPoint = tr2.left.end1;
                target = null;
                for (Boundary b : left.boundaries){
                    for (Arc a : b.arcs){
                        if (Point.distance(tr2.base.end1, a.end1) < 0.001){
                            target = a;
                            break;
                        }
                    }
                }
                tr2.right = target;
                tp.tr1 = tr1;
                tp.tr2 = tr2;
                return;
            }

            Point circle = new Point(0, 0, 0);
            double radius = computeIntersectionCircle(leftL.owner.sphere.center, rightL.owner.sphere.center, circle, SesConfig.probeRadius);


            //Boundary[] bs = ConcavePatchUtil.generateCircularBoundary(leftL.owner, rightL.owner, circle, radius);
            Vector circleN = Point.subtractPoints(leftL.owner.sphere.center, circle).makeUnit();

            Plane cPlane = new Plane(circle, circleN);
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

            Vector cusp1To0 = Point.subtractPoints(cusps[0], cusps[1]).makeUnit();
            Vector perpendi = Vector.getNormalVector(circleN, cusp1To0).makeUnit();
            Plane p = new Plane(cusps[1], perpendi);
            Plane ro = new Plane(circle, perpendi);

            /*if (Math.abs(Math.abs(Point.subtractPoints(tp.convexPatchArcs.get(0).owner.sphere.center, tp.convexPatchArcs.get(1).owner.sphere.center).makeUnit().dotProduct(cusp1To0)) - 1) > 0.001){
                System.out.println("not parallel vectors bro");
            }*/

            //Point[] cusps = Util.getCuspPoints(leftL.vrts, leftL.center, rightL.center, SesConfig.probeRadius);

            if (leftL.owner.id == 2337 || rightL.owner.id == 2337){
                System.out.println("americja");
            }
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
            ArcUtil.refineArc(leftStart, Main.maxEdgeLen, true,1, false);
            ArcUtil.refineArc(leftStart, Main.maxEdgeLen, false, 0, false);
            leftStart.endEdge1 = new Edge(0, 1);
            leftStart.endEdge1.p1 = leftStart.end1;
            leftStart.endEdge1.p2 = leftStart.vrts.get(1);
            leftStart.endEdge2 = new Edge(leftStart.vrts.size() - 2, leftStart.vrts.size() - 1);
            leftStart.endEdge2.p1 = leftStart.vrts.get(leftStart.vrts.size() - 2);
            leftStart.endEdge2.p2 = leftStart.end2;
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
            ArcUtil.refineArc(leftEnd, Main.maxEdgeLen, true, 1, false);
            ArcUtil.refineArc(leftEnd, Main.maxEdgeLen, false, 0, false);
            leftEnd.endEdge1 = new Edge(0, 1);
            leftEnd.endEdge1.p1 = leftEnd.end1;
            leftEnd.endEdge1.p2 = leftEnd.vrts.get(1);
            leftEnd.endEdge2 = new Edge(leftEnd.vrts.size() - 2, leftEnd.vrts.size() - 1);
            leftEnd.endEdge2.p1 = leftEnd.vrts.get(leftEnd.vrts.size() - 2);
            leftEnd.endEdge2.p2 = leftEnd.end2;
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
            ArcUtil.refineArc(rightStart, Main.maxEdgeLen, true,1, false);
            ArcUtil.refineArc(rightStart, Main.maxEdgeLen, false,0, false);
            rightStart.endEdge1 = new Edge(0, 1);
            rightStart.endEdge1.p1 = rightStart.end1;
            rightStart.endEdge1.p2 = rightStart.vrts.get(1);
            rightStart.endEdge2 = new Edge(rightStart.vrts.size() - 2, rightStart.vrts.size() - 1);
            rightStart.endEdge2.p1 = rightStart.vrts.get(rightStart.vrts.size() - 2);
            rightStart.endEdge2.p2 = rightStart.end2;
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
            ArcUtil.refineArc(rightEnd, Main.maxEdgeLen, true,1, false);
            ArcUtil.refineArc(rightEnd, Main.maxEdgeLen, false,0, false);
            rightEnd.endEdge1 = new Edge(0, 1);
            rightEnd.endEdge1.p1 = rightEnd.end1;
            rightEnd.endEdge1.p2 = rightEnd.vrts.get(1);
            rightEnd.endEdge2 = new Edge(rightEnd.vrts.size() - 2, rightEnd.vrts.size() - 1);
            rightEnd.endEdge2.p1 = rightEnd.vrts.get(rightEnd.vrts.size() - 2);
            rightEnd.endEdge2.p2 = rightEnd.end2;
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
            List<Point> middlevrts;
            Vector toS = Point.subtractPoints(cusps[0], circle).makeUnit();
            Vector toE = Point.subtractPoints(cusps[1], circle).makeUnit();
            Vector n = Vector.getNormalVector(toE, toS).makeUnit();
            if (n.dotProduct(circleN) > 0.0){
                //System.out.println("guns n roses");
                middlevrts = ArcUtil.generateCircArc(cusps[0], cusps[1], circle, radius, 2, true);
            } else {
                middlevrts = ArcUtil.generateCircArc(cusps[0], cusps[1], circle, radius, 2, false);
            }
            Arc leftMid = new Arc(circle, radius);
            leftMid.owner = leftL.owner;
            //leftMid.end1 = cusps[0];
            //leftMid.end2 = cusps[1];
            leftMid.setEndPoints(cusps[0], cusps[1], false);
            leftMid.setNormal(Point.subtractPoints(leftL.owner.sphere.center, circle).makeUnit());
            leftMid.prev = leftStart;
            leftStart.next = leftMid;
            leftMid.next = leftEnd;
            leftEnd.prev = leftMid;
            leftMid.vrts.addAll(middlevrts);
            leftMid.endEdge1 = new Edge(0, 1);
            leftMid.endEdge1.p1 = leftMid.end1;
            leftMid.endEdge1.p2 = leftMid.vrts.get(1);
            leftMid.endEdge2 = new Edge(leftMid.vrts.size() - 2, leftMid.vrts.size() - 1);
            leftMid.endEdge2.p1 = leftMid.vrts.get(leftMid.vrts.size() - 2);
            leftMid.endEdge2.p2 = leftMid.end2;
            //leftMid.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
            ArcUtil.refineArc(leftMid, Main.maxEdgeLen, false,0, false);
            //Util.reverserOrder(leftMid, true);
            //leftMid.buildEdges();





            Arc rightMid = new Arc(circle, radius);
            rightMid.owner = rightL.owner;
            //rightMid.end1 = cusps[0];
            //rightMid.end2 = cusps[1];
            rightMid.setEndPoints(cusps[0], cusps[1], false);
            rightMid.setNormal(Point.subtractPoints(rightMid.owner.sphere.center, circle).makeUnit());
            rightMid.vrts.addAll(leftMid.vrts);
            //Util.reverserOrder(rightMid, true);
            ArcUtil.reverseArc(rightMid, true);
            rightMid.mid = leftMid.mid;
            rightMid.next = rightEnd;
            rightEnd.prev = rightMid;
            rightMid.prev = rightStart;
            rightStart.next = rightMid;
            rightMid.endEdge1 = new Edge(0, 1);
            rightMid.endEdge1.p1 = rightMid.end1;
            rightMid.endEdge1.p2 = rightMid.vrts.get(1);
            rightMid.endEdge2 = new Edge(rightMid.vrts.size() - 2, rightMid.vrts.size() - 1);
            rightMid.endEdge2.p1 = rightMid.vrts.get(rightMid.vrts.size() - 2);
            rightMid.endEdge2.p2 = rightMid.end2;
            //rightMid.buildEdges();




            /*leftStart.buildEdges();
            leftMid.buildEdges();
            leftEnd.buildEdges();

            rightStart.buildEdges();
            rightMid.buildEdges();
            rightEnd.buildEdges();*/
            ArcUtil.buildEdges(leftStart);
            ArcUtil.buildEdges(leftMid);
            ArcUtil.buildEdges(leftEnd);
            ArcUtil.buildEdges(rightStart);
            ArcUtil.buildEdges(rightMid);
            ArcUtil.buildEdges(rightEnd);

            leftStart.endEdge1.prev = leftL.endEdge1.prev;
            leftStart.endEdge1.prev.next = leftStart.endEdge1;
            leftEnd.endEdge2.next = leftL.endEdge2.next;
            leftEnd.endEdge2.next.prev = leftEnd.endEdge2;
            rightStart.endEdge1.prev = rightL.endEdge1.prev;
            rightStart.endEdge1.prev.next = rightStart.endEdge1;
            rightEnd.endEdge2.next = rightL.endEdge2.next;
            rightEnd.endEdge2.next.prev = rightEnd.endEdge2;
            leftMid.endEdge1.prev = leftStart.endEdge2;
            leftStart.endEdge2.next = leftMid.endEdge1;
            leftMid.endEdge2.next = leftEnd.endEdge1;
            leftEnd.endEdge1.prev = leftMid.endEdge2;
            rightMid.endEdge1.prev = rightStart.endEdge2;
            rightStart.endEdge2.next = rightMid.endEdge1;
            rightMid.endEdge2.next = rightEnd.endEdge1;
            rightEnd.endEdge1.prev = rightMid.endEdge2;

            SphericalPatch left = leftL.owner;
            SphericalPatch right = rightL.owner;

            if (left.intersectingPatches.contains(right.id) || right.intersectingPatches.contains(left.id)){
                System.out.println("found al double trimming " + left.id + " " + right.id);
            }

            left.neighbours.add(right);
            right.neighbours.add(left);

            leftStart.owner = leftMid.owner = leftEnd.owner = left;
            rightStart.owner = rightMid.owner = rightEnd.owner = right;
            leftMid.intersecting = true;
            rightMid.intersecting = true;
            Main.intersectingArcs.add(leftMid);
            Main.intersectingArcs.add(rightMid);
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
            rightEnd.cuspTriangle = leftStart.cuspTriangle = tp.tr1;
            if (rightEnd.vrts.size() != leftStart.vrts.size()){
                System.out.println(" ");
            }
            tp.tr1.cuspPoint = cusps[0];

            tp.tr2.base = (tp.tr1.base == tp.convexPatchArcs.get(0)) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);
            tp.tr2.left = leftEnd;
            tp.tr2.right = rightStart;
            leftEnd.cuspTriangle = rightStart.cuspTriangle = tp.tr2;
            if (leftEnd.vrts.size() != rightStart.vrts.size()){
                System.out.println(" ");
            }
            tp.tr2.cuspPoint = cusps[1];
            leftStart.torus = leftEnd.torus = rightStart.torus = rightEnd.torus = tp;
            //Plane circlePlane = new Plane(cusps[0], circleN);
            //List<Edge[]> nextIntersections = rp.getIntersectionPoints(leftEnd.next, bs[0], circlePlane);
            //List<Edge[]> nextIntersections =
            /*if (nextIntersections.size() > 0){
                System.out.println(" ");
            }*/

            /*if (nextIntersections.size() > 0){
                System.out.println(" ");
            }*/
            /*ConcavePatchArc s = leftEnd.next;
            while (s != leftStart){
                List<Edge[]> prieseky = Util.looploopIntersection(leftMid, circlePlane, s);
                if (prieseky.size() > 0){
                    System.out.println(" ");
                }
                s = s.next;
            }
            s = rightEnd.next;
            while (s != rightStart){
                List<Edge[]> prieseky = Util.looploopIntersection(rightMid, circlePlane, s);
                if (prieseky.size() > 0){
                    System.out.println(" ");
                }
                s = s.next;
            }*/
            /*if (nextIntersections.size() == 2){
                Edge f = nextIntersections.get(0)[0];
                Edge g = nextIntersections.get(0)[1];
                Vector n = Vector.getNormalVector(circleN, Point.subtractPoints(g.p2, g.p1)).makeUnit();
                Plane r = new Plane(g.p1, n);
                Point proj1 = r.pointProjection(f.p1);
                Point proj2 = r.pointProjection(f.p2);
                Edge k = new Edge(0, 0);
                k.p1 = proj1;
                k.p2 = proj2;
                Point i = Util.computeIntersection(g, k);
                System.out.println(" ");
                f = nextIntersections.get(1)[0];
                g = nextIntersections.get(1)[1];
                n = Vector.getNormalVector(circleN, Point.subtractPoints(g.p2, g.p1)).makeUnit();
                r = new Plane(g.p1, n);
                proj1 = r.pointProjection(f.p1);
                proj2 = r.pointProjection(f.p2);
                k.p1 = proj1;
                k.p2 = proj2;
                Point j = Util.computeIntersection(g, k);
                System.out.println(" ");
            }
            nextIntersections = rp.getIntersectionPoints(leftStart.prev, bs[0], circlePlane);
            nextIntersections = rp.getIntersectionPoints(rightEnd.next, bs[1], circlePlane);
            if (nextIntersections.size() > 0){
                System.out.println( " ");
            }
            nextIntersections = rp.getIntersectionPoints(rightStart.prev, bs[1], circlePlane);
            if (nextIntersections.size() > 0){
                System.out.println(" ");
            }*/
        } catch (Exception e){
            System.err.println("tp id: " + tp.id);
            e.printStackTrace();
        }
    }

    public static void processIntersectingArcsOnPatch(Arc arc){
        if (!arc.intersecting || !arc.valid){
            return;
        }
        List<Point> pointOfIntersection = new ArrayList<>();
        Point lastPoI = null;
        boolean found = false;
        SphericalPatch sp = arc.owner;
        Vector u1 = Point.subtractPoints(arc.end1, arc.center).makeUnit();
        Vector u2 = Point.subtractPoints(arc.end2, arc.center).makeUnit();
        Vector n1 = Point.subtractPoints(sp.sphere.center, arc.center).makeUnit();

        Plane p1 = new Plane(arc.center, n1);
        for (Boundary b : sp.boundaries){
            for (Arc k : b.arcs){
                if (k == arc || k == arc.prev || k == arc.next){
                    continue;
                }
                boolean allInside = k.vrts.stream().allMatch(p -> p1.checkPointLocation(p) > 0.0);
                if (allInside){
                    continue;
                }
                Vector v1 = Point.subtractPoints(k.end1, k.center).makeUnit();
                Vector v2 = Point.subtractPoints(k.end2, k.center).makeUnit();
                Plane p2 = new Plane(k.center, v1, v2);
                Vector dir = new Vector(0, 0, 0);
                Point pint = new Point(0, 0, 0);
                if (Plane.getIntersectionLine(p1, p2, dir, pint)){
                    Vector hypo = Point.subtractPoints(arc.center, pint);
                    dir.makeUnit();
                    double odvesna = dir.dotProduct(hypo);
                    double dist = Math.sqrt(hypo.dotProduct(hypo) - Math.pow(odvesna, 2));
                    if (dist - arc.radius < 0.0){
                        Point midTetiva = Point.translatePoint(pint, Vector.scaleVector(dir, odvesna));
                        double odv2 = Math.sqrt(Math.pow(arc.radius, 2) - Math.pow(dist, 2));
                        Point in1 = Point.translatePoint(midTetiva, Vector.scaleVector(dir, odv2));
                        Point in2 = Point.translatePoint(midTetiva, Vector.scaleVector(dir, -odv2));
                            /*if ((k.isInside(in1) || k.isInside(in2)) && (cpl.isInside(in1) || cpl.isInside(in2))) {
                                System.out.println("possible intermezzo for " + cpl.owner.id);
                            }*/
                        if ((k.isInside(in1) && arc.isInside(in1)) || (k.isInside(in2) && arc.isInside(in2))){
                            System.out.println("INTERMEZZO for " + arc.owner.id);
                            found = true;
                        }
                        if (k.isInside(in1) && arc.isInside(in1)){
                                /*if (lastPoI == null || Point.distance(lastPoI, in1) > 0.001){
                                    lastPoI = in1;
                                    pointOfIntersection.add(in1);
                                }*/
                            if (!pointOfIntersection.stream().anyMatch(p -> Point.distance(p, in1) < 0.001)){
                                pointOfIntersection.add(in1);
                            }
                        }
                        if (k.isInside(in2) && arc.isInside(in2)){
                                /*if (lastPoI == null || Point.distance(lastPoI, in2) > 0.001){
                                    lastPoI = in2;
                                    pointOfIntersection.add(in2);
                                }*/
                            if (!pointOfIntersection.stream().anyMatch(p -> Point.distance(p, in2) < 0.001)){
                                pointOfIntersection.add(in2);
                            }
                        }
                    }
                }
            }
        }
        if (pointOfIntersection.size() == 1){
            System.out.println("111");
        }
        if (found && pointOfIntersection.size() == 2) {
                /*System.out.println("For " + cpl.owner.id + " found " + pointOfIntersection.size() + " pois");
                for (Point p : pointOfIntersection){
                    System.out.println(p.toString());
                }*/
            for (Boundary b : sp.boundaries){
                for (Arc a : b.arcs){
                    a.valid = false;
                }
            }
            Point e1 = pointOfIntersection.get(0);
            Point e2 = Point.translatePoint(e1, Vector.scaleVector(arc.normal, 5));
            Point f1 = pointOfIntersection.get(1);
            Point f2 = Point.translatePoint(f1, Vector.scaleVector(arc.normal, 5));
            Edge ed1 = new Edge(0, 0);
            ed1.p1 = e1;
            ed1.p2 = e2;
            Edge ed2 = new Edge(0, 0);
            ed2.p1 = f1;
            ed2.p2 = f2;
            List<Boundary> newBs = new ArrayList<>();
            Boundary b = new Boundary();
            b.patch = sp;
            Arc start = arc.next.next;
            Arc a = start;
            do {
                Plane p = new Plane(a.center, a.normal);
                boolean trim = false;
                Point in = null;
                for (Point i : pointOfIntersection){
                    if (Math.abs(p.checkPointLocation(i)) < 0.001 && a.isInside(i)){
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
                    ArcUtil.refineArc(newA2, Main.maxEdgeLen, false,0, false);
                    Arc newA = new Arc(arc.center, arc.radius);
                    newA.setNormal(arc.normal);
                    newA.vrts.add(in);
                    newA.vrts.add(arc.end2);
                    //newA.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    newA.setEndPoints(in, arc.end2, false);
                    ArcUtil.refineArc(newA, Main.maxEdgeLen, false,0, false);

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
                    pointOfIntersection.remove(in);
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
            List<Arc> newArcs = new ArrayList<>();
            start = arc.prev.prev;
            a = start;
            do {
                Plane p = new Plane(a.center, a.normal);
                boolean trim = false;
                Point in = null;
                for (Point i : pointOfIntersection){
                    if (Math.abs(p.checkPointLocation(i)) < 0.001 && a.isInside(i)){
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
                    a.mid = null;
                    //a.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    ArcUtil.refineArc(a, Main.maxEdgeLen, false,0, false);
                    a.setEndPoints(in, a.end2, false);
                    a.valid = true;
                    arc.vrts.clear();
                    arc.vrts.add(arc.end1);
                    arc.vrts.add(in);
                    arc.mid = null;
                    //arc.refineLoop(Main.maxEdgeLen, 0.0, false, 0, false);
                    arc.setEndPoints(arc.end1, in, false);
                    ArcUtil.refineArc(arc, Main.maxEdgeLen, false,0, false);

                    a.prev = arc;
                    arc.next = a;
                    arc.valid = true;
                    //a.buildEdges();
                    //cpl.buildEdges();
                    newArcs.add(a);
                    newArcs.add(arc);
                    pointOfIntersection.remove(in);
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
        pointOfIntersection.clear();
        lastPoI = null;
    }

    public static void meshToroidalPatch(ToroidalPatch tp){
        try {
            /*if ((!circular && concavePatchArcs.size() < 2) || !concavePatchArcs.get(0).valid || !concavePatchArcs.get(1).valid){
                valid = false;
                Main.rectangles.remove(this);
                return;
            }*/
            if (!tp.circular){
                if (tp.concavePatchArcs.size() < 2){
                    tp.valid = false;
                    return;
                } else {
                    if (!tp.concavePatchArcs.get(0).valid || !tp.concavePatchArcs.get(1).valid){
                        tp.valid = false;
                        return;
                    }
                }
            }
            if (tp.tr1 != null){
                List<Point> top = new ArrayList<>();
                for (int i = 0; i < tp.tr1.base.vrts.size(); ++i){
                    top.add(tp.tr1.cuspPoint);
                }
                Arc left = new Arc(tp.tr1.left.center, tp.tr1.left.radius);
                left.vrts.addAll(tp.tr1.left.vrts);
                ArcUtil.reverseArc(left, true);
                Arc topL = new Arc(tp.tr1.cuspPoint, 0);
                topL.vrts.addAll(top);
                meshToroidalPatch(tp, tp.tr1.base, topL, left, tp.tr1.right, true);

                top.clear();
                for (int i = 0; i < tp.tr2.base.vrts.size(); ++i){
                    top.add(tp.tr2.cuspPoint);
                }
                left = new Arc(tp.tr2.left.center, tp.tr2.left.radius);
                left.vrts.addAll(tp.tr2.left.vrts);
                ArcUtil.reverseArc(left, true);
                topL.vrts.clear();
                topL.vrts.addAll(top);
                meshToroidalPatch(tp, tp.tr2.base, topL, left, tp.tr2.right, true);
                return;
            }
            Arc bottom = tp.convexPatchArcs.get(0);
            Arc left = null;
            Arc right = null;
            Arc top = tp.convexPatchArcs.get(1);

            Vector a1toa2 = Point.subtractPoints(tp.convexPatchArcs.get(0).owner.sphere.center, tp.convexPatchArcs.get(1).owner.sphere.center).makeUnit();
            Vector a1toprobe = Point.subtractPoints(tp.probe1, tp.convexPatchArcs.get(0).owner.sphere.center);
            a1toa2.multiply(a1toa2.dotProduct(a1toprobe));
            Point centerOfRot = Point.translatePoint(tp.convexPatchArcs.get(0).owner.sphere.center, a1toa2);
            Vector vr = Point.subtractPoints(tp.probe1, centerOfRot);
            if (tp.concavePatchArcs.size() == 0){
                if (vr.sqrtMagnitude() < Double.longBitsToDouble(Main.probeRadius.get())) {
                    System.err.println("beginning to mesh circ patch");
                    Point centerOfRect = Point.translatePoint(tp.probe1, Point.subtractPoints(tp.probe2, tp.probe1).multiply(0.5f));
                    double centerToCuspLength = Math.sqrt(Math.pow(Double.longBitsToDouble(Main.probeRadius.get()), 2) - Math.pow(Point.subtractPoints(tp.probe1, tp.probe2).sqrtMagnitude() / 2.f, 2));
                    Point bottomCusp = Point.translatePoint(centerOfRect, Point.subtractPoints(bottom.owner.sphere.center, top.owner.sphere.center).makeUnit().multiply(centerToCuspLength));
                    Point topCusp = Point.translatePoint(centerOfRect, Point.subtractPoints(top.owner.sphere.center, bottom.owner.sphere.center).makeUnit().multiply(centerToCuspLength));
                    Arc topForBottomRect = new Arc(bottom.center, bottom.radius);
                    for (Point p : bottom.vrts) {
                        topForBottomRect.vrts.add(bottomCusp);
                    }
                    Point newCenter = (Point.subtractPoints(bottom.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), bottom.owner.sphere)).sqrtMagnitude() < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(bottom.end2);
                    Point mid;
                    Vector v;
                    left.vrts.add(bottomCusp);
                    left.setEndPoints(bottom.end2, bottomCusp, true);
                    ArcUtil.refineArc(left, Main.maxEdgeLen, true,3, false);
                    ArcUtil.refineArc(left, Main.maxEdgeLen, false,0, false);

                    newCenter = (Point.subtractPoints(left.center, tp.probe1).sqrtMagnitude() < 0.0001) ? tp.probe2 : tp.probe1;

                    right = new Arc(newCenter, SesConfig.probeRadius);
                    right.vrts.add(bottom.end1);
                    mid = Point.translatePoint(right.vrts.get(0), Point.subtractPoints(bottomCusp, right.vrts.get(0)).multiply(0.5f));
                    v = Point.subtractPoints(mid, right.center).makeUnit().multiply(right.radius);

                    right.vrts.add(bottomCusp);
                    right.setEndPoints(bottom.end1, bottomCusp, true);
                    int numOfDivs = (int)(Math.log10(left.vrts.size() - 1) / Math.log10(2));
                    ArcUtil.refineArc(right, Main.maxEdgeLen, true, numOfDivs, false);
                    meshToroidalPatch(tp, bottom, topForBottomRect, left, right, false);

                    Arc bottomForTopRect = new Arc(top.center, top.radius);
                    for (Point p : top.vrts){
                        bottomForTopRect.vrts.add(topCusp);
                    }
                    newCenter = (Point.subtractPoints(top.end2, Sphere.getContactPoint(new Sphere(tp.probe1, Double.longBitsToDouble(Main.probeRadius.get())), top.owner.sphere)).sqrtMagnitude() < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(top.end2);
                    mid = Point.translatePoint(left.vrts.get(0), Point.subtractPoints(topCusp, left.vrts.get(0)).multiply(0.5f));
                    v = Point.subtractPoints(mid, left.center).makeUnit().multiply(left.radius);
                    left.vrts.add(topCusp);
                    left.setEndPoints(top.end2, topCusp, true);
                    ArcUtil.refineArc(left, Main.maxEdgeLen, true,3, false);
                    ArcUtil.refineArc(left, Main.maxEdgeLen, false, 0, false);
                    newCenter = (Point.subtractPoints(left.center, tp.probe1).sqrtMagnitude() < 0.0001) ? tp.probe2 : tp.probe1;
                    right = new Arc(newCenter, SesConfig.probeRadius);
                    right.vrts.add(top.end1);
                    mid = Point.translatePoint(right.vrts.get(0), Point.subtractPoints(topCusp, right.vrts.get(0)).multiply(0.5f));
                    v = Point.subtractPoints(mid, right.center).makeUnit().multiply(right.radius);

                    right.vrts.add(topCusp);
                    right.setEndPoints(top.end1, topCusp, true);
                    numOfDivs = (int)(Math.log10(left.vrts.size() - 1) / Math.log10(2));
                    ArcUtil.refineArc(right, Main.maxEdgeLen, true, numOfDivs, false);
                    meshToroidalPatch(tp, top, bottomForTopRect, left, right, false);
                    System.out.println("finished meshing circ patch");
                } else {

                    Point newCenter = (Point.subtractPoints(bottom.end2, Sphere.getContactPoint(new Sphere(tp.probe1, SesConfig.probeRadius), bottom.owner.sphere)).sqrtMagnitude() < 0.0001) ? tp.probe1 : tp.probe2;
                    left = new Arc(newCenter, SesConfig.probeRadius);
                    left.vrts.add(bottom.end2);
                    left.vrts.add(top.end1);
                    left.setEndPoints(bottom.end2, top.end1, true);
                    ArcUtil.refineArc(left, Main.maxEdgeLen, true,3, false);
                    newCenter = (Point.subtractPoints(left.center, tp.probe1).sqrtMagnitude() < 0.0001) ? tp.probe2 : tp.probe1;
                    right = new Arc(newCenter, SesConfig.probeRadius);

                    right.vrts.add(bottom.end1);
                    right.vrts.add(top.end2);
                    right.setEndPoints(bottom.end1, top.end2, true);
                    ArcUtil.refineArc(right, Main.maxEdgeLen, false,3, false);
                    if (right.vrts.size() != left.vrts.size()){
                        System.out.println("weird");
                    }
                    meshToroidalPatch(tp, bottom, top, left, right, false);
                    tp.circleMeshed = true;
                }
            } else {
                Vector toProbe = Point.subtractPoints(tp.probe1, bottom.owner.sphere.center).makeUnit().multiply(bottom.owner.sphere.radius + SesConfig.probeRadius);
                Vector atom1ToAtom2 = Point.subtractPoints(top.owner.sphere.center, bottom.owner.sphere.center).makeUnit();
                atom1ToAtom2.multiply(toProbe.dotProduct(atom1ToAtom2));
                double probeToRotationAx = -42;
                probeToRotationAx = getProbeAxisDistance(tp.probe1, top.owner.sphere.center, bottom.owner.sphere.center);
                if (probeToRotationAx - SesConfig.probeRadius < 0.0){

                } else {
                    left = (Point.subtractPoints(bottom.end2, tp.concavePatchArcs.get(0).end2).sqrtMagnitude() < 0.0001) ? tp.concavePatchArcs.get(0) : tp.concavePatchArcs.get(1);
                    ArcUtil.reverseArc(left, true);
                    right = (left == tp.concavePatchArcs.get(0)) ? tp.concavePatchArcs.get(1) : tp.concavePatchArcs.get(0);
                    meshToroidalPatch(tp, bottom, top, left, right, false);
                    ArcUtil.reverseArc(left, true);
                }
            }
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    private static void meshToroidalPatch(ToroidalPatch tp, Arc bottom, Arc top, Arc left, Arc right, boolean special){
        try {
            List<Point> leftVArc = new ArrayList<>();
            leftVArc.addAll(left.vrts);
            List<Point> rightVArc = new ArrayList<>();
            Point currProbe = null;
            Point prevProbe = left.center;
            for (int i = 1; i < bottom.vrts.size(); ++i) {
                Point vert = bottom.vrts.get(bottom.vrts.size() - i - 1);
                Vector toProbe = Point.subtractPoints(vert, bottom.owner.sphere.center).makeUnit().multiply(SesConfig.probeRadius);
                currProbe = Point.translatePoint(vert, toProbe);
                rightVArc.clear();
                if (i == bottom.vrts.size() - 1) {
                    rightVArc.addAll(right.vrts);
                    for (Point bod : rightVArc){
                        if (bod == null){
                            System.out.println(" ");
                        }
                    }
                } else {
                    if (!special) {
                        rightVArc.add(bottom.vrts.get(bottom.vrts.size() - i - 1));
                        /*Point mid = Point.translatePoint(top.vrts.get(i), Point.subtractPoints(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i)).multiply(0.5f));
                        Vector v = Point.subtractPoints(mid, currProbe).makeUnit().multiply(Double.longBitsToDouble(Main.probeRadius.get()));
                        mid = Point.translatePoint(currProbe, v);
                        newHelp.add(mid);*/
                        rightVArc.add(top.vrts.get(i));
                        //newHelp = Util.refineLoop(newHelp, currProbe, Double.longBitsToDouble(Main.probeRadius.get()), Main.maxEdgeLen);
                        rightVArc = ArcUtil.generateCircArc(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i), currProbe, SesConfig.probeRadius, left.vrts.size() - 1, false);
                        if (rightVArc.size() != left.vrts.size()){
                            System.out.println("incorrect num of verts");
                        }
                        for (Point bod : rightVArc){
                            if (bod == null){
                                System.out.println(" ");
                            }
                        }
                    } else {
                        List<Point> vrts = ArcUtil.generateCircArc(bottom.vrts.get(bottom.vrts.size() - i - 1), top.vrts.get(i), currProbe, SesConfig.probeRadius, left.vrts.size() - 1, false);
                        if (vrts == null){
                            System.out.println("this is null");
                            rightVArc.add(bottom.vrts.get(bottom.vrts.size() - i -1));
                            rightVArc.add(top.vrts.get(i));
                        } else {
                            rightVArc.addAll(vrts);
                            if (vrts.size() != left.vrts.size()){
                                System.err.println("vrts.size != left.vrts.size()");
                            }
                        }
                    }
                }
                for (int j = 0; j < left.vrts.size() - 1; ++j) {
                    Point newPoint = null;
                    /*if (i < bottom.vrts.size() - 1) {
                        if (j < left.vrts.size() - 2) {
                            Vector v = Point.subtractPoints(top.vrts.get(i), bottom.vrts.get(bottom.vrts.size() - i - 1)).multiply((float)(j + 1) / (float)(left.vrts.size() - 1));
                            newPoint = Point.translatePoint(bottom.vrts.get(bottom.vrts.size() - i - 1), v);
                            v = Point.subtractPoints(newPoint, currProbe).makeUnit().multiply(Double.longBitsToDouble(Main.probeRadius.get()));
                            newPoint = Point.translatePoint(currProbe, v);
                        } else {
                            newPoint = top.vrts.get(i);
                        }
                        newHelp.add(newPoint);
                    }*/
                    //float[] color = (i < bottom.vrts.size() - 2) ? green : blue;
                    int offset = tp.vertices.size();
                    /*for (Point p : vertices){
                        if (p.idx < 0){
                            offset++;
                        }
                    }*/
                    tp.vrts.add(leftVArc.get(j)); // = 0
                    Vector n = Point.subtractPoints(prevProbe, leftVArc.get(j)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(leftVArc.get(j)); // = 0
                    tp.normals.add(n);

                    //vrts.add(new Point(color));
                    tp.vrts.add(rightVArc.get(j)); // = 1
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(currProbe, rightVArc.get(j)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(rightVArc.get(j));
                    tp.normals.add(n);

                    tp.vrts.add(leftVArc.get(j + 1)); // = 2
                    //vrts.add(new Point(color));
                    if (prevProbe == null || leftVArc.get(j + 1) == null){
                        System.out.println(" ");
                    }
                    n = Point.subtractPoints(prevProbe, leftVArc.get(j + 1)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(leftVArc.get(j + 1));
                    tp.normals.add(n);

                    tp.vrts.add(leftVArc.get(j + 1)); // = 2
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(prevProbe, leftVArc.get(j + 1)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));
                    tp.vrts.add(rightVArc.get(j)); // = 1
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(currProbe, rightVArc.get(j)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));
                    tp.vrts.add(rightVArc.get(j + 1)); // = 3
                    //vrts.add(new Point(color));
                    n = Point.subtractPoints(currProbe, rightVArc.get(j + 1)).makeUnit();
                    tp.vrts.add(new Point(n.getFloatData()));

                    tp.vertices.add(rightVArc.get(j + 1));
                    tp.normals.add(n);
                    tp.faces.add(new Face(offset, offset + 1, offset + 2));
                    tp.faces.add(new Face(offset + 2, offset + 1, offset + 3));
                    Main.numoftriangles += 2;

                }
                leftVArc.clear();
                leftVArc.addAll(rightVArc);
                prevProbe = currProbe;
            }
        } catch (Exception e){
            e.printStackTrace();
            System.err.println("tp id: " + tp.id);
        }
    }

    public static double computeIntersectionCircle(Point probe1, Point probe2, Point result, double probeRadius){
        Vector halfway = Point.subtractPoints(probe1, probe2).multiply(0.5f);
        Point center = Point.translatePoint(probe2, halfway);
        result.x = center.x;
        result.y = center.y;
        result.z = center.z;
        return Math.sqrt(Math.pow(probeRadius, 2) - Math.pow(halfway.sqrtMagnitude(), 2));
    }

    public static Point computeCusp(Point probe, Sphere a1, Sphere a2){
        Vector axis = Point.subtractPoints(a2.center, a1.center).makeUnit();
        Vector a1toprobe = Point.subtractPoints(probe, a1.center);
        double atomToProbeLength = a1.radius + SesConfig.probeRadius;
        a1toprobe.makeUnit();
        double alpha = Math.acos(axis.dotProduct(a1toprobe));
        double beta = Math.asin(((atomToProbeLength) * Math.sin(alpha)) / SesConfig.probeRadius);
        double gama = Math.PI - alpha - beta;
        double atomToCusp = (SesConfig.probeRadius * Math.sin(gama)) / Math.sin(alpha);
        axis.multiply(atomToCusp);
        return Point.translatePoint(a1.center, axis);
    }

    public static void processIntersectingConcavePatches(){
        for (SphericalPatch sp : Main.triangles){
            trimConcavePatch(sp);
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
            ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
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
        ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
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
                ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
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
        int kratky = shortArcs(b);
        return in1Arc.bOwner;
    }

    private static void boundaryAlgorithm2(Plane circle, double radius, Point in1, Arc a1, Point in2, Arc a2, List<Point> intersectionPoints, List<Point> usedPoints, List<Boundary> toRemove, List<Boundary> newBS, SphericalPatch sp){
        Point pStart = in1;
        List<Boundary> toRemove2 = new ArrayList<>();
        Boundary b = new Boundary();
        b.patch = sp;
        Arc start = a1;
        Arc a = a2;

        boolean toBridge = (a1 != a2 || circle.checkPointLocation(a1.end1) > 0.0);
        boolean forceContinue = false;
        boolean arcExit = false;
        int it = 0;
        //boolean forceBridge = (circle.checkPointLocation(a1.end1) < 0.0 && circle.checkPointLocation(a1.end2) < 0.0);
        do {
            if (toBridge){ //|| forceBridge){
                Arc newA = new Arc(circle.p, radius);
                newA.setEndPoints(in1, in2, false);
                newA.setNormal(circle.v);
                newA.vrts.add(in1);
                newA.vrts.add(in2);
                usedPoints.add(in1);
                usedPoints.add(in2);
                ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
                b.arcs.add(newA);
                toBridge = false;
                in1 = in2;
                a1 = a2;
                in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
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
                    ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
                    toBridge = true;
                    in1 = in2;
                    in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                    a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
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
                    if (a.cuspTriangle != null){
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
                        ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
                    }
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
                    if (a.cuspTriangle != null){
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
                        ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
                    }
                    b.arcs.add(newA);
                }
                a = a.next;
                forceContinue = false;
                toBridge = true;
                in1 = in2;
                a1 = a2;
                in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                while (a2.bOwner != a1.bOwner && !a2.bOwner.nestedBoundaries.contains(a1.bOwner)){
                    in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in2, false, circle.p, circle.v, true);
                    a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                }
            }else {
                Arc newA = ArcUtil.dbgCloneArc(a);
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
        } while (forceContinue || in1 != pStart);
        ArcUtil.buildEdges(b, true);
        newBS.add(b);
        int kratky  = shortArcs(b);
        for (Boundary b_ : toRemove2){
            if (!toRemove.contains(b_)){
                toRemove.add(b_);
            }
        }
        //return toRemove;
        //return a1.bOwner;
    }

    private static int id = -1002;
    private static void generateNewBoundaries2(SphericalPatch sp, List<Point> intersectionPs, Plane circle, double radius){
        try {
            if (intersectionPs.size() % 2 == 1){
                return;
            }
            List<Point> intersectionPoints = new ArrayList<>(intersectionPs);
            List<Point> usedPoints = new ArrayList<>();
            if (intersectionPoints.size() > 1) {
                List<Boundary> newBS = new ArrayList<>();
                List<Boundary> toRemove = new ArrayList<>();
                int i = 0;
                while (i < intersectionPoints.size()) {
                    Point in1 = findOptimalPoint(intersectionPoints, usedPoints, sp, circle);
                    if (in1 == null){
                        break;
                    }
                    Point in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                    Point pStart = in1;

                    Arc a1 = ArcUtil.findContainingArc(in1, circle, sp, null);
                    Arc a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                    final Point p1 = in1;
                    final Point p2 = in2;

                    if (a1.bOwner != a2.bOwner && !a1.bOwner.nestedBoundaries.contains(a2.bOwner)){
                        return;
                    }
                    if (a1.bOwner != a2.bOwner && a1.bOwner.nestedBoundaries.contains(a2.bOwner)) {
                        System.out.println("about to merge nested boundaries");
                    }

                    //int spl = patchSplit(intersectionPoints, in1, in2, sp, circle);
                    //List<Point> ps = (intersectionPoints.size() > 2) ?  patchSplit(intersectionPoints, in1, in2, sp, circle) : intersectionPoints;
                    List<Point> ps = (intersectionPoints.size() > 2) ? getUsablePoints(intersectionPoints, in1, in2, sp, circle) : intersectionPoints;
                    /*if (spl){
                        System.out.println("will split");
                    } else {
                        System.out.println("wont split");
                    }*/



                    if (Point.distance(in1, a1.end2) < 0.001 && Point.distance(in2, a2.end2) < 0.001){
                        boolean allInside = true;
                        for (Arc a : a1.bOwner.arcs){
                            allInside = allInside && a.vrts.stream().allMatch(p -> (Point.distance(p, p1) < 0.001 || Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0));
                        }
                        if (allInside){
                            return;
                        }
                    }

                    if (a1.next == a2.prev && Point.distance(in1, a1.end2) < 0.001 && Point.distance(in2, a2.end1) < 0.001){
                        //System.err.println(sp.id + " a1n == a2p " + intersectionPoints.size());
                        if (a1.next.vrts.stream().allMatch(p -> (Point.distance(p, p1) < 0.001 || Point.distance(p, p2) < 0.001 || circle.checkPointLocation(p) > 0.0))){
                            return;
                        }
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
                    System.out.println("to : " + sp.id);
                    //toRemove.add(boundaryAlgorithm2(circle, radius, in1, a1, in2, a2, ps, usedPoints, newBS, sp));
                    boundaryAlgorithm2(circle, radius, in1, a1, in2, a2, ps, usedPoints, toRemove, newBS, sp);
                    i += ps.size();
                    //System.out.println("did: " + sp.id);
                    //sp.boundaries.remove(a.bOwner);f
                }
                /*sp.boundaries.removeAll(toRemove);
                sp.boundaries.addAll(newBS);*/
                updatePatchBoundaries(sp, toRemove, newBS);
                boolean check = checkBoundaryOwn(sp);
            }
        } catch (Exception e){
            e.printStackTrace();
            System.err.println("for: " + sp.id);
        }
    }

    private static void generateNewBoundaries(SphericalPatch sp, List<Point> intersectionPs, Plane circle, double radius){
        try {
            List<Point> intersectionPoints = new ArrayList<>(intersectionPs);
            List<Point> usedPoints = new ArrayList<>();
            if (intersectionPoints.size() > 1) {
                List<Boundary> newBS = new ArrayList<>();
                List<Boundary> toRemove = new ArrayList<>();
                int i = 0;
                while (i < intersectionPoints.size()) {
                    Point in1 = findOptimalPoint(intersectionPoints, usedPoints, sp, circle);
                    //intersectionPoints.remove(in1);
                    usedPoints.add(in1);
                    i++;
                    Point in2 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in1, false, circle.p, circle.v, true);
                    //intersectionPoints.remove(in2);
                    usedPoints.add(in2);
                    i++;

                    Arc a1 = ArcUtil.findContainingArc(in1, circle, sp, null);
                    Arc a2 = ArcUtil.findContainingArc(in2, circle, sp, null);
                    if (a1.next == a2.prev && Point.distance(in1, a1.end2) < 0.001 && Point.distance(in2, a2.end1) < 0.001){
                        System.err.println(sp.id + " a1n == a2p " + intersectionPoints.size());
                        if (a1.next.vrts.stream().allMatch(p -> (Point.distance(p, in1) < 0.001 || Point.distance(p, in2) < 0.001 || circle.checkPointLocation(p) > 0.0))){
                            return;
                        }
                    }
                    toRemove.add(a1.bOwner);
                    toRemove.add(a2.bOwner);

                    if (a1.bOwner != a2.bOwner){
                        System.out.println("same but different but same");
                        return;
                    }

                    Boundary b = new Boundary();

                    Arc newA1 = new Arc(a1.center, a1.radius);
                    newA1.vrts.add(a1.end1);
                    newA1.vrts.add(in1);
                    newA1.setEndPoints(a1.end1, in1, false);
                    newA1.setNormal(a1.normal);
                    ArcUtil.refineArc(newA1, Main.maxEdgeLen, false, 0, false);
                    newA1.prev = a1.prev;
                    newA1.prev.next = newA1;

                    Arc bridge = new Arc(circle.p, radius);
                    bridge.setEndPoints(in1, in2, false);
                    bridge.setNormal(circle.v);
                    bridge.vrts.add(in1);
                    if (Vector.getNormalVector(bridge.toEnd1, bridge.toEnd2).makeUnit().dotProduct(bridge.normal) < 0.0){
                        Point mid = Point.translatePoint(in1, Point.subtractPoints(in2, in1).multiply(0.5f));
                        Vector v = Point.subtractPoints(circle.p, mid).makeUnit().multiply(radius);
                        mid = Point.translatePoint(circle.p, v);
                        bridge.vrts.add(mid);
                    }
                    bridge.vrts.add(in2);
                    ArcUtil.refineArc(bridge, Main.maxEdgeLen, false, 0, false);

                    Point in3 = ArcUtil.findClosestPointOnCircle(intersectionPoints, in2, false, circle.p, circle.v, true);
                    Arc a3  = (in3 == null) ? null : ArcUtil.findContainingArc(in3, circle, sp, null);
                    Arc arc3 = null;

                    if (a3 == a2){
                        if (Vector.getNormalVector(a3.toEnd1, a3.toEnd2).makeUnit().dotProduct(a3.normal) < 0.0){
                            Point mid = Point.translatePoint(a3.end1, Point.subtractPoints(a3.end2, a3.end1).multiply(0.5f));
                            Vector v = Point.subtractPoints(a3.center, mid).makeUnit().multiply(a3.radius);
                            mid = Point.translatePoint(a3.center, v);
                            Vector toIn2 = Point.subtractPoints(in2, a2.center).makeUnit();
                            Vector toIn3 = Point.subtractPoints(in3, a2.center).makeUnit();
                            if (Vector.getNormalVector(a2.toEnd1, toIn2).makeUnit().dotProduct(a2.normal) > 0.0 && Vector.getNormalVector(a2.toEnd1, toIn3).makeUnit().dotProduct(a2.normal) > 0.0){
                                double alpha1 = Math.acos(a2.toEnd1.dotProduct(toIn2));
                                double alpha2 = Math.acos(a2.toEnd1.dotProduct(toIn3));
                                if (alpha1 - alpha2 < 0.0){
                                    arc3 = new Arc(a2.center, a2.radius);
                                    arc3.setEndPoints(in2, in3, false);
                                    arc3.setNormal(a2.normal);
                                    arc3.vrts.add(in2);
                                    if (Vector.getNormalVector(arc3.toEnd1, arc3.toEnd2).makeUnit().dotProduct(arc3.normal) < 0.0){
                                        Point mid2 = Point.translatePoint(arc3.end1, Point.subtractPoints(arc3.end2, arc3.end1).multiply(0.5f));
                                        Vector v2 = Point.subtractPoints(arc3.center, mid2).makeUnit().multiply(arc3.radius);
                                        mid2 = Point.translatePoint(arc3.center, v2);
                                        arc3.vrts.add(mid2);
                                    }
                                    arc3.vrts.add(in3);
                                    ArcUtil.refineArc(arc3, Main.maxEdgeLen, false, 0, false);
                                    //b.arcs.add(arc3);
                                    //arc3.prev = bridge;
                                    //bridge.next = arc3;
                                }
                            } else if (Vector.getNormalVector(a2.toEnd1, toIn2).makeUnit().dotProduct(a2.normal) > 0.0 && Vector.getNormalVector(a2.toEnd1, toIn3).makeUnit().dotProduct(a2.normal) < 0.0){
                                arc3 = new Arc(a2.center, a2.radius);
                                arc3.setEndPoints(in2, in3, false);
                                arc3.setNormal(a2.normal);
                                arc3.vrts.add(in2);
                                if (Vector.getNormalVector(arc3.toEnd1, arc3.toEnd2).makeUnit().dotProduct(arc3.normal) < 0.0){
                                    Point mid2 = Point.translatePoint(arc3.end1, Point.subtractPoints(arc3.end2, arc3.end1).multiply(0.5f));
                                    Vector v2 = Point.subtractPoints(arc3.center, mid2).makeUnit().multiply(arc3.radius);
                                    mid2 = Point.translatePoint(arc3.center, v2);
                                    arc3.vrts.add(mid2);
                                }
                                arc3.vrts.add(in3);
                                ArcUtil.refineArc(arc3, Main.maxEdgeLen, false, 0, false);
                                //b.arcs.add(arc3);
                                //arc3.prev = bridge;
                                //bridge.next = arc3;
                            }
                        } else {
                            double alpha1 = Math.acos(a2.toEnd1.dotProduct(Point.subtractPoints(in2, a2.center).makeUnit()));
                            double alpha2 = Math.acos(a2.toEnd1.dotProduct(Point.subtractPoints(in3, a2.center).makeUnit()));
                            if (alpha1 - alpha2 < 0.0){
                                arc3 = new Arc(a2.center, a2.radius);
                                arc3.setEndPoints(in2, in3, false);
                                arc3.setNormal(a2.normal);
                                arc3.vrts.add(in2);
                                if (Vector.getNormalVector(arc3.toEnd1, arc3.toEnd2).makeUnit().dotProduct(arc3.normal) < 0.0){
                                    Point mid2 = Point.translatePoint(arc3.end1, Point.subtractPoints(arc3.end2, arc3.end1).multiply(0.5f));
                                    Vector v2 = Point.subtractPoints(arc3.center, mid2).makeUnit().multiply(arc3.radius);
                                    mid2 = Point.translatePoint(arc3.center, v2);
                                    arc3.vrts.add(mid2);
                                }
                                arc3.vrts.add(in3);
                                ArcUtil.refineArc(arc3, Main.maxEdgeLen, false, 0, false);
                                //b.arcs.add(arc3);
                                //arc3.prev = bridge;
                                //bridge.next = arc3;
                            }
                        }
                    }
                    bridge.prev = newA1;
                    newA1.next = bridge;
                    if (arc3 == null) {
                        b.arcs.add(newA1);
                        b.arcs.add(bridge);

                        if (Point.distance(in2, a2.end2) > 0.001) {
                            Arc newA2 = new Arc(a2.center, a2.radius);
                            newA2.vrts.add(in2);
                            newA2.vrts.add(a2.end2);
                            newA2.setEndPoints(in2, a2.end2, false);
                            newA2.setNormal(a2.normal);
                            ArcUtil.refineArc(newA2, Main.maxEdgeLen, false, 0, false);
                            b.arcs.add(newA2);
                        }

                        /*newA2.prev = bridge;
                        bridge.next = newA2;
                        newA2.next = a2.next;
                        newA2.next.prev = newA2;*/


                        //b.arcs.add(newA2);
                    } else {
                        usedPoints.add(in3);
                        i++;
                        Point in4 = ArcUtil.findClosestPointOnCircle(intersectionPoints, arc3.end2, false, circle.p, circle.v, true);
                        usedPoints.add(in4);
                        i++;
                        Arc a4 = ArcUtil.findContainingArc(in4, circle, sp, null);

                        Arc arc4 = new Arc(circle.p, radius);
                        arc4.setEndPoints(in3, in4, false);
                        arc4.setNormal(circle.v);
                        arc4.vrts.add(in3);
                        arc4.vrts.add(in4);
                        ArcUtil.refineArc(arc4, Main.maxEdgeLen, false, 0, false);

                        Arc arc5 = new Arc(a4.center, a4.radius);
                        arc5.setEndPoints(in4, a4.end2, false);
                        arc5.setNormal(a4.normal);
                        arc5.vrts.add(in4);
                        arc5.vrts.add(a4.end2);
                        ArcUtil.refineArc(arc5, Main.maxEdgeLen, false, 0, false);

                        b.arcs.add(newA1);
                        b.arcs.add(bridge);
                        b.arcs.add(arc3);
                        b.arcs.add(arc4);
                        b.arcs.add(arc5);

                        bridge.next = arc3;
                        arc3.prev = bridge;
                        arc3.next = arc4;
                        arc4.prev = arc3;
                        arc4.next = arc5;
                        arc5.prev = arc4;
                        arc5.next = a4.next;
                        arc5.next.prev = arc5;
                        arc3 = arc5;
                        //a2 = arc5.next;
                    }
                    int shortage = shortArcs(b);
                    Arc start = newA1;
                    Arc a = (arc3 == null) ? a2.next : arc3.next;
                    while (a != start) {
                        b.arcs.add(a);
                        a = a.next;
                    }
                    b.patch = sp;
                    ArcUtil.buildEdges(b, true);
                    ArcUtil.linkArcs(b.arcs);
                    newBS.add(b);
                }
                sp.boundaries.removeAll(toRemove);
                sp.boundaries.addAll(newBS);
                toRemove.clear();
            }
        } catch (Exception e){
            System.out.println("with sp.id = " + sp.id);
            e.printStackTrace();
            int a = 1+1;

        }
    }

    private static Map<Integer, List<Plane>> planes = new TreeMap<>();

    private static void trimConcavePatch(SphericalPatch sp){
        try {
            List<Neighbor<double[], SphericalPatch>> neighbors = new ArrayList<>();
            Main.probeTree.range(sp.sphere.center.getData(), 2 * SesConfig.probeRadius, neighbors);
            Collections.sort(neighbors, new Comparator<Neighbor<double[], SphericalPatch>>() {
                @Override
                public int compare(Neighbor<double[], SphericalPatch> o1, Neighbor<double[], SphericalPatch> o2) {
                    if (Math.abs(o1.distance - o2.distance) < 0.001) {
                        return 0;
                    }
                    return (o1.distance - o2.distance > 0.0) ? -1 : 1;
                }
            });
            //List<Integer> laterId = new ArrayList<>();
            //List<Point> laterCirc = new ArrayList<>();
            List<Intersection> possibleIntersections = new ArrayList<>();
            for (Neighbor<double[], SphericalPatch> n : neighbors) {
                if (n.value == sp) {
                    continue;
                }
                SphericalPatch sp2 = n.value;
                if (sp.intersectingPatches.contains(sp2.id)) {
                    continue;
                }
                /*if (sp.id == 97 || sp.id == 98 || sp.id == 99 || sp2.id == 97 || sp2.id == 98 || sp2.id == 99) {
                    continue;
                }*/
                Point center = new Point(0, 0, 0);
                double radius = computeIntersectionCircle(sp.sphere.center, sp2.sphere.center, center, SesConfig.probeRadius);
                Vector nV = Point.subtractPoints(sp.sphere.center, center).makeUnit();
                if (!planes.containsKey(sp.id)) {
                    planes.put(sp.id, new ArrayList<>());
                }
                /*if (!planes.containsKey(sp2.id)) {
                    planes.put(sp2.id, new ArrayList<>());
                }*/
                sp.intersectingPatches.add(sp2.id);
                if (Point.distance(sp.sphere.center, sp2.sphere.center) < 0.005){
                    continue;
                }
                //sp2.intersectingPatches.add(sp.id);
                if (sp2.id == 1187 && sp.id == 1196){
                    System.out.println("1299 and 1371");
                }

                if (sp.id == 1371){
                    System.out.println("am.");
                }
                Plane p = new Plane(center, nV);
                List<Point> intersectionPoints = new ArrayList<>();
                findIntersectionPoints(sp, center, radius, intersectionPoints, null);
                if (intersectionPoints.size() > 1) {
                    if (planes.get(sp.id).stream().noneMatch(plane -> plane.isIdenticalWith(p))) {
                        planes.get(sp.id).add(p);
                        System.out.println("about to: " + sp.id);
                        generateNewBoundaries2(sp, intersectionPoints, p, radius);
                    }
                } else if (intersectionPoints.size() == 0) {
                    //laterId.add(sp2.id);
                    //laterCirc.add(center);
                    /*Intersection in = new Intersection();
                    in.sp1 = sp;
                    in.sp2 = sp2;
                    in.circleRadius = radius;
                    in.circle = p;
                    possibleIntersections.add(in);*/
                    Boundary newB = ArcUtil.generateCircularBoundary(p, radius);
                    boolean nest = false;
                    List<Boundary> removeFromSP = new ArrayList<>();
                    for (Boundary b : sp.boundaries){
                        boolean isInside = true;
                        for (Arc a : b.arcs){
                            Plane rho = new Plane(a.center, a.normal);
                            //isInside = isInside && rho.checkPointLocation(possibleIntersections.get(i).circle.p) > 0.0;
                            isInside = isInside && newB.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
                        }
                        if (isInside){
                            //Intersection in = possibleIntersections.get(i);
                            //Boundary newB = ArcUtil.generateCircularBoundary(in.circle, in.circleRadius);

                            //b.nestedBoundaries.add(newB);
                            for (Boundary nb : b.nestedBoundaries){
                                for (Arc a : nb.arcs){
                                    Plane rho = new Plane(a.center, a.normal);
                                    isInside = isInside && newB.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
                                }
                            }
                            if (isInside) {
                                newB.nestedBoundaries.add(b);
                                newB.nestedBoundaries.addAll(b.nestedBoundaries);
                                for (Boundary nb : newB.nestedBoundaries) {
                                    nb.nestedBoundaries.add(newB);
                                }
                                List<Boundary> toRemove = new ArrayList<>();
                                for (Boundary nb : newB.nestedBoundaries){
                                    for (Arc a : nb.arcs){
                                        if (!a.vrts.stream().allMatch(v -> p.checkPointLocation(v) > 0.0)){
                                            toRemove.add(nb);
                                            break;
                                        }
                                    }
                                }
                                newB.nestedBoundaries.removeAll(toRemove);
                                removeFromSP.addAll(toRemove);
                                for (Boundary nb : newB.nestedBoundaries){
                                    nb.nestedBoundaries.removeAll(toRemove);
                                }
                                nest = true;
                                System.out.println("nestted b " + sp.id);
                            }
                        }
                    }
                    if (nest){
                        newB.patch = sp;
                        sp.boundaries.removeAll(removeFromSP);
                        sp.boundaries.add(newB);
                    }

                }
            }

            /*for (int i = 0; i < possibleIntersections.size(); ++i){
                for (Boundary b : sp.boundaries){
                    boolean isInside = true;
                    for (Arc a : b.arcs){
                        Plane rho = new Plane(a.center, a.normal);
                        isInside = isInside && rho.checkPointLocation(possibleIntersections.get(i).circle.p) > 0.0;
                    }
                    if (isInside){
                        Intersection in = possibleIntersections.get(i);
                        Boundary newB = ArcUtil.generateCircularBoundary(in.circle, in.circleRadius);
                        b.nestedBoundaries.add(newB);
                        newB.nestedBoundaries.add(b);
                        sp.boundaries.add(newB);
                        System.out.println("nestted b " + sp.id);
                        return;
                    }
                }
            }*/
                /*intersectionPoints.clear();
                findIntersectionPoints(sp2, center, radius, intersectionPoints, null);
                p.changePlaneOrientation(p.v.multiply(-1));
                if (intersectionPoints.size() > 1) {
                    if (planes.get(sp2.id).stream().noneMatch(plane -> plane.isIdenticalWith(p))) {
                        planes.get(sp2.id).add(p);
                        generateNewBoundaries(sp2, intersectionPoints, p, radius);
                    }
                }*/
            //System.out.println("");
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    private static void findIntersectionPoints(SphericalPatch sp, Point circle, double radius, List<Point> intersectionPoints, Arc exclude){
        Vector n = Point.subtractPoints(sp.sphere.center, circle).makeUnit();
        Plane p = new Plane(circle, n);
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
                if (exclude != null && (a == exclude || a == exclude.next || a == exclude.prev)){
                        continue;
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
                Plane p2 = new Plane(a.center, a.normal);
                Vector vInt = new Vector(0,0,0);
                Point pInt = new Point(0,0,0);
                if (Plane.getIntersectionLine(p, p2, vInt, pInt)){
                    Vector hypo = Point.subtractPoints(circle, pInt);
                    double odvesna = vInt.dotProduct(hypo);
                    double dist = Math.sqrt(hypo.dotProduct(hypo) - odvesna * odvesna);
                    if (dist - radius < 0.0){
                        Point midTetiva = Point.translatePoint(pInt, Vector.scaleVector(vInt, odvesna));
                        double odv2 = Math.sqrt(radius * radius - dist * dist);
                        Point in1 = Point.translatePoint(midTetiva, Vector.scaleVector(vInt, odv2));
                        Point in2 = Point.translatePoint(midTetiva, Vector.scaleVector(vInt, -odv2));
                        if (a.isInside(in1)){
                            if (intersectionPoints.stream().noneMatch(v -> Point.distance(in1, v) < 0.005)){
                                intersectionPoints.add(in1);
                            }
                        }
                        if (a.isInside(in2)) {
                            if (intersectionPoints.stream().noneMatch(v -> Point.distance(in2, v) < 0.005)){
                                intersectionPoints.add(in2);
                            }
                        }
                    }
                }
            }
        }
    }

    private static Point findOptimalPoint(List<Point> points, List<Point> usedPoints, SphericalPatch sp, Plane plane){
        try {
            for (Point p : points) {
                if (usedPoints.contains(p)){
                    continue;
                }
                Arc a = ArcUtil.findContainingArc(p, plane, sp, null);
                Optional<Point> opsecondPointOnArc = points.stream().filter(point -> point != p && a.isInside(point)).findFirst(); //if there is more than one point on the arc a(more specifically 2 is the upper bound)
                if (opsecondPointOnArc.isPresent()) {
                    Point secondPoint = opsecondPointOnArc.get();
                    Vector fV = Point.subtractPoints(p, a.center).makeUnit();
                    Vector sV = Point.subtractPoints(secondPoint, a.center).makeUnit();
                    double alpha1 = Math.acos(fV.dotProduct(a.toEnd1));
                    alpha1 = (Vector.getNormalVector(a.toEnd1, fV).makeUnit().dotProduct(a.normal) < 0.0) ? 2 * Math.PI - alpha1 : alpha1;
                    double alpha2 = Math.acos(Point.subtractPoints(secondPoint, a.center).makeUnit().dotProduct(a.toEnd1));
                    alpha2 = (Vector.getNormalVector(a.toEnd1, sV).makeUnit().dotProduct(a.normal) < 0.0) ? 2 * Math.PI - alpha2 : alpha2;
                    //return (alpha1 - alpha2 < 0.0) ? p : secondPoint;
                    if (alpha1 - alpha2 < 0.0 && !usedPoints.contains(p)){
                        return p;
                    }
                    if (alpha1 - alpha2 > 0.0 && !usedPoints.contains(secondPoint)){
                        return  secondPoint;
                    }
                } else {
                    if (Point.distance(p, a.end1) < 0.001 && plane.checkPointLocation(a.end2) > 0.0){
                        continue;
                    }
                    if (plane.checkPointLocation(a.end1) > 0.0 && !usedPoints.contains(p)) {
                        return p;
                    }
                }
            }
        } catch (Exception e){
            e.printStackTrace();
        }
        return null;
    }

    public static void trimSelfIntersectingPatch(Arc arc){
        try {
            if (!arc.valid || !arc.intersecting){
                return;
            }
            SphericalPatch sp = arc.owner;
            if (sp.id == 658){
                System.out.println("ligh");
            }
            List<Point> intersectionPoints = new ArrayList<>();
            findIntersectionPoints(sp, arc.center, arc.radius, intersectionPoints, arc);
            List<Point> invalid = new ArrayList<>();
            for (Point v : intersectionPoints){
                if (Point.distance(v, arc.prev.end1) < 0.0015 || Point.distance(v, arc.next.end2) < 0.0015){
                    invalid.add(v);
                }
            }
            intersectionPoints.removeAll(invalid);
            if (intersectionPoints.size() > 1){ //theoretically at most 2 intersection points should be encountered(and in most cases), 1 point is special case which is not handled right now(it is rare)
                for (Boundary b : sp.boundaries){
                    for (Arc a : b.arcs){
                        a.valid = false;
                    }
                }
                Plane circle = new Plane(arc.center, arc.normal);
                List<Boundary> newBS = new ArrayList<>();
                Point first = ArcUtil.findClosestPointOnCircle(intersectionPoints, arc.end1, true, arc.center, arc.normal, true);
                Arc a = ArcUtil.findContainingArc(first, circle, sp, arc);
                Boundary b = new Boundary();
                Arc newA = (Point.distance(first, arc.end1) < 0.0018) ? null : new Arc(arc.center, arc.radius);
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
                        ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
                    }
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
                    ArcUtil.refineArc(newA2, Main.maxEdgeLen, false, 0, false);
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
                intersectionPoints.remove(first);

                Point second = intersectionPoints.get(0); //remaining intersection point
                a = ArcUtil.findContainingArc(second, circle, sp, arc);
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
                    ArcUtil.refineArc(newA, Main.maxEdgeLen, false, 0, false);
                }
                /*newA.prev = a.prev;
                newA.prev.next = newA;*/

                newA2 = (Point.distance(second, arc.end2) < 0.0018) ? null : new Arc(arc.center, arc.radius);
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
                        ArcUtil.refineArc(newA2, Main.maxEdgeLen, false, 0, false);
                    }
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

    private static List<Point> patchSplit(List<Point> intersectionPoints, Point origin, Point start, SphericalPatch sp, Plane circle){
        boolean init = true;
        Point end = null;
        Arc startArc = null;
        Arc endArc = null;
        Arc originArc = ArcUtil.findContainingArc(origin, circle, sp, null);
        Arc a = null;
        List<Point> usablePoints = new ArrayList<>(intersectionPoints);
        List<Point> usedPoints = new ArrayList<>();
        //usablePoints.remove(origin);
        //usedPoints.add(origin);
        Point predecessor = ArcUtil.findClosestPointOnCircle(intersectionPoints, origin, false, circle.p, circle.v, false);
        int involvedPoints = 2;
        do {

            if (init) {
                usablePoints.remove(start);
                usedPoints.add(start);
                end = ArcUtil.findClosestPointOnCircle(usablePoints, start, false, circle.p, circle.v, true);
                startArc = ArcUtil.findContainingArc(start, circle, sp, null);
                endArc = ArcUtil.findContainingArc(end, circle, sp, null);
                if (startArc.bOwner != endArc.bOwner){
                    return usedPoints;
                }
                involvedPoints++;
                a = (startArc == endArc) ? ((ArcUtil.getOrder(startArc, start, end) < 0) ? startArc : startArc.next) : startArc.next;
                init = false;

            } else {
                for (Point p : usablePoints){
                    if (a.isInside(p)) {
                        start = p;
                        init = true;
                        break;
                    }
                }
                if (!init) {
                    a = a.next;
                }
            }

            if (a == endArc){
                if (endArc == originArc){
                    return usedPoints;
                }
                start = end;
                init = true;
            }

            if (a == originArc){
                return usedPoints;
            }


        } while (true);
    }

    private static List<Point> getUsablePoints(List<Point> intersectionPoints, Point origin, Point start, SphericalPatch sp, Plane circle){
        List<Point> usedPoints = new ArrayList<>();
        List<Point> usablePoints = new ArrayList<>(intersectionPoints);
        Arc originArc = ArcUtil.findContainingArc(origin, circle, sp, null);
        Arc startArc = ArcUtil.findContainingArc(start, circle, sp, null);
        Arc endArc = null;
        Arc a = null;
        Point end = null;
        boolean init = true;
        boolean bridge = (circle.checkPointLocation(startArc.end1) > 0.0);
        do {
            if (init){
                usedPoints.add(start);
                usablePoints.remove(start);
                /*if (start == origin){
                    return usedPoints;
                }*/
                startArc = ArcUtil.findContainingArc(start, circle, sp, null);
                end = ArcUtil.findClosestPointOnCircle(intersectionPoints, start, false, circle.p, circle.v, true);
                endArc = ArcUtil.findContainingArc(end, circle, sp, null);
                a = (startArc == endArc) ? ((ArcUtil.getOrder(startArc, start, end) < 0) ? startArc : startArc.next) : startArc.next;
                if (startArc.bOwner != endArc.bOwner && startArc.bOwner.nestedBoundaries.contains(endArc.bOwner)){
                    a = endArc;
                }
                if (end == origin){
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
                for (Point p : usablePoints){
                    if (a.isInside(p)){
                        final Arc arc = a;
                        Optional<Point> otherP = usablePoints.stream().filter(v -> v != p && arc.isInside(v)).findFirst();
                        newStart = (otherP.isPresent()) ? ((ArcUtil.getOrder(a, p, otherP.get()) < 0) ? p : otherP.get()) : p;
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
        return usedPoints;
    }

    private static boolean checkBoundaryOwn(SphericalPatch sp){
        for (Boundary b : sp.boundaries){
            for (Arc a : b.arcs){
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
        for (Boundary b : toRemove){
            for (Boundary nb : b.nestedBoundaries){
                nb.nestedBoundaries.remove(b);
            }
        }
        for (Boundary b : toRemove){
            if (b.mergeSplit.size() == 1){
                Boundary b2 = b.mergeSplit.get(0);
                if (b2.mergeSplit.size() == 1){
                    for (Boundary nb : b.nestedBoundaries){
                        if (areNested(b2, nb)){
                            b2.nestedBoundaries.add(nb);
                            nb.nestedBoundaries.add(b2);
                        } else {
                            sp.boundaries.remove(nb);
                            for (Boundary nb_ : nb.nestedBoundaries){
                                nb_.nestedBoundaries.remove(nb);
                            }
                        }
                    }
                } else if (b2.mergeSplit.size() > 1){
                    for (Boundary nb : b.nestedBoundaries){
                        if (areNested(b2, nb)){
                            b2.nestedBoundaries.add(nb);
                            nb.nestedBoundaries.add(b2);
                        } else {
                            sp.boundaries.remove(nb);
                            for (Boundary nb_ : nb.nestedBoundaries){
                                nb_.nestedBoundaries.remove(nb);
                            }
                        }
                    }

                }
                b2.mergeSplit.stream().forEach(b_ -> b_.mergeSplit.clear());
            } else if (b.mergeSplit.size() > 1){
                for (Boundary newB : b.mergeSplit){
                    for (Boundary nb : b.nestedBoundaries){
                        if (areNested(newB, nb)){
                            newB.nestedBoundaries.add(nb);
                            nb.nestedBoundaries.add(newB);
                        } else {
                            sp.boundaries.remove(nb);
                            for (Boundary nb_ : nb.nestedBoundaries){
                                nb_.nestedBoundaries.remove(nb);
                            }
                        }
                    }
                    newB.mergeSplit.clear();
                }
                b.mergeSplit.clear();
            }
        }
        toAdd.stream().forEach(b -> b.mergeSplit.clear());
        sp.boundaries.removeAll(toRemove);
        sp.boundaries.addAll(toAdd);
    }

    private static boolean areNested(Boundary b1, Boundary b2){
        boolean nest = b1.arcs.stream().allMatch(a -> {
            Plane rho = new Plane(a.center, a.normal);
            return b2.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
        });
        if (nest){
            nest = b2.arcs.stream().allMatch(a -> {
                Plane rho = new Plane(a.center, a.normal);
                return b1.vrts.stream().allMatch(v -> rho.checkPointLocation(v) > 0.0);
            });
            if (!nest){
                return false;
            }
        } else {
            return false;
        }
        return true;
    }
}