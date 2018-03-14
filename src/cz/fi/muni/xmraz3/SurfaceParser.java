package cz.fi.muni.xmraz3;

import com.jogamp.opengl.math.Quaternion;
import com.jogamp.opengl.math.VectorUtil;
import cz.fi.muni.xmraz3.math.Plane;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Sphere;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.mesh.*;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import smile.neighbor.KDTree;

import java.io.*;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Created by radoslav on 27.2.2017.
 */
public class SurfaceParser {

    private static Map<Integer, Map<Integer, List<SphericalPatch>>> missingArcs = new TreeMap<>();

    private static void updateMissingArcs(int atom1, int atom2, SphericalPatch triangle){
        int smaller = (atom1 > atom2) ? atom2 : atom1;
        int bigger = (smaller == atom1) ? atom2 : atom1;
        if (!missingArcs.containsKey(smaller)){
            missingArcs.put(smaller, new TreeMap<>());
        }
        Map<Integer, List<SphericalPatch>> mapList = missingArcs.get(smaller);
        if (!mapList.containsKey(bigger)){
            mapList.put(bigger, new ArrayList<>());
        }
        mapList.get(bigger).add(triangle);
    }

    private static void constructProbeTree(){
        double[][] keys = new double[Surface.triangles.size()][3];
        SphericalPatch values[] = new SphericalPatch[Surface.triangles.size()];
        for (int i = 0; i < keys.length; ++i){
            Point p = Surface.triangles.get(i).sphere.center;
            keys[i] = p.getData();
            values[i] = Surface.triangles.get(i);
        }
        Surface.probeTree = new KDTree<>(keys, values);
    }

    public static void ses_start(String folder) {

        Runnable r1 = new Runnable() {
            @Override
            public void run() {
                MeshRefinement.convexEdgeSplitMap = new ArrayList<>(SesConfig.atomCount);
                for (int i = 0; i < SesConfig.atomCount; ++i){
                    MeshRefinement.convexEdgeSplitMap.add(new TreeMap<>());
                }
            }
        };
        (new Thread(r1)).start();
        Runnable r2 = new Runnable() {
            @Override
            public void run() {
                MeshRefinement.concaveEdgeSplitMap = new ArrayList<>(SesConfig.trianglesCount);
                for (int i = 0; i < SesConfig.trianglesCount; ++i){
                    MeshRefinement.concaveEdgeSplitMap.add(new TreeMap<>());
                }
            }
        };
        (new Thread(r2)).start();
        Surface.triangles.clear();
        Surface.rectangles.clear();
        Surface.atomsProcessed.set(0);
        Surface.selfIntersectingRects.clear();
        Surface.intersectingArcs.clear();
        Surface.numoftriangles = 0;
        SphericalPatch.nextConcaveID = SphericalPatch.nextConvexID = 0;
        ToroidalPatch.nextID = 0;
        Surface.triangles.ensureCapacity(SesConfig.trianglesCount);
        Surface.rectangles.ensureCapacity(SesConfig.toriCount);
        Surface.convexPatches = SurfaceParser.parseAtoms(folder + "atoms.dat");
        Surface.centerOfgravity.x /= SesConfig.atomCount;
        Surface.centerOfgravity.y /= SesConfig.atomCount;
        Surface.centerOfgravity.z /= SesConfig.atomCount;
        Surface.probeRadius.set(Double.doubleToLongBits(SesConfig.probeRadius));
        SurfaceParser.parseConvexAndToriPatches(folder + "rectangles.dat");

        try {
            SurfaceParser.parseTriangles(folder + "triangles.dat");
            constructProbeTree();
            Surface.probeTree.setIdenticalExcluded(true);


            PatchUtil.processSelfIntersectingTori();
            PatchUtil.processSelfIntersectingConcavePatches();
            PatchUtil.processIntersectingConcavePatches();

            fillCommonVertices();
        } catch (Exception e){
            System.out.println(e.getMessage());
        }
    }

    private static void fillCommonVertices(){
        Surface.commonVrts.clear();
        int idx = 1;
        for (SphericalPatch a : Surface.convexPatches){
            for (Boundary b : a.boundaries){
                for (Point p : b.vrts){
                    p.idx = idx++;
                    Surface.commonVrts.add(p);
                    Vector n = Point.subtractPoints(p, a.sphere.center).makeUnit();
                    Surface.normals.add(n);
                    p.common = true;
                }
            }
        }
        for (SphericalPatch cp : Surface.triangles){
            for (Point p : cp.boundaries.get(0).vrts){
                p.idx = idx++;
                Surface.commonVrts.add(p);
                Vector n = Point.subtractPoints(cp.sphere.center, p).makeUnit();
                Surface.normals.add(n);
                p.common = true;
            }
        }
    }

    public static String loadFile(String filename) {
        FileInputStream in = null;
        try{
            in = new FileInputStream(filename);
        } catch (FileNotFoundException e)
        {
            System.out.println(e.getMessage());
        }
        StringBuilder sb = new StringBuilder();
        Reader r = new InputStreamReader(in);
        int ch;
        try{
            while ((ch = in.read()) != -1){
                sb.append((char) ch);
            }
        } catch (IOException e) {
            System.out.println(e.getMessage());
        }
        return sb.toString();
    }

    public static void parseSesConfig(String raw){
        try {
            JSONParser parser = new JSONParser();
            JSONObject obj = (JSONObject)parser.parse(raw);
            SesConfig.probeRadius = Surface.scaleFactor * (double)obj.get("ProbeRadius");
            SesConfig.atomCount = ((Long)obj.get("AtomsCount")).intValue();
            SesConfig.toriCount = ((Long)obj.get("TorusCount")).intValue();
            SesConfig.trianglesCount = ((Long)obj.get("TrianglesCount")).intValue();
        } catch (ParseException e){
            e.printStackTrace();
        }
    }

    public static List<SphericalPatch> parseForAtoms(String raw){
        ArrayList<SphericalPatch> atList = new ArrayList<>();
        try{
            JSONParser parser = new JSONParser();
            Object obj = parser.parse(raw);
            JSONArray ar = (JSONArray)obj;
            for (Object oj : ar){
                JSONObject at = (JSONObject)oj;
                JSONObject atom = (JSONObject)at.get("atom");
                double x = (double)atom.get("x");
                double y = (double)atom.get("y");
                double z = (double)atom.get("z");
                double r = (double)atom.get("r");
                Surface.centerOfgravity.x += x;
                Surface.centerOfgravity.y += y;
                Surface.centerOfgravity.z += z;
                long atId = (long)at.get("id");
                SphericalPatch spatch = new SphericalPatch(new Point(atom), Surface.scaleFactor * r, true);
                //atMap.put(atId, ahatom);
                atList.add(spatch);
            }
        } catch (ParseException e){
            System.err.println("atom parsing error");
        }
        return atList;
    }
    static int rollingCount = 0;
    public static void jsonParseConvexAndToriPatches(String raw, List<SphericalPatch> atoms, double pRadius){
        try{
            JSONParser parser = new JSONParser();
            Object obj = parser.parse(raw);
            JSONArray ar = (JSONArray)obj;
            for (Object oj : ar){
                JSONObject entry = (JSONObject)oj;
                int atom1Id = ((Long)entry.get("atom1Id")).intValue();
                int atom2Id = ((Long)entry.get("atom2Id")).intValue();
                if (atom1Id == 2556 && atom2Id == 385){

                }
                SphericalPatch atom1 = atoms.get((int)atom1Id);
                SphericalPatch atom2 = atoms.get((int)atom2Id);
                JSONObject jProbe1 = (JSONObject)entry.get("probe1");
                JSONObject jProbe2 = (JSONObject)entry.get("probe2");
                JSONObject jProbeMid = (JSONObject)entry.get("center");
                Sphere probe1 = new Sphere(new Point(jProbe1), pRadius);
                Sphere probe2 = new Sphere(new Point(jProbe2), pRadius);
                Sphere probeMid = new Sphere(new Point(jProbeMid), pRadius);

                constructConvexPatchArc(atom1, atom2, probe1, probe2, probeMid);

                rollingCount += 2;
            }
        } catch (ParseException e){
            System.err.println(e.getMessage());
        }
    }

    public static void parseConvexAndToriPatches(String filename){
        int atom1Id, atom2Id;
        try (DataInputStream in = new DataInputStream(new FileInputStream(filename))){
            byte[] buffer = new byte[SesConfig.toriCount * 44];
            in.read(buffer, 0, buffer.length);
            ByteBuffer data = ByteBuffer.wrap(buffer);
            for (int i = 0; i < SesConfig.toriCount; ++i){
                atom1Id = data.getInt();
                atom2Id = data.getInt();
                Sphere centerProbe = new Sphere(new Point(data.getFloat(), data.getFloat(), data.getFloat()), SesConfig.probeRadius);
                Sphere probe1 = new Sphere(new Point(data.getFloat(), data.getFloat(), data.getFloat()), SesConfig.probeRadius);
                Sphere probe2 = new Sphere(new Point(data.getFloat(), data.getFloat(), data.getFloat()), SesConfig.probeRadius);
                SphericalPatch atom1 = Surface.convexPatches.get(atom1Id);
                SphericalPatch atom2 = Surface.convexPatches.get(atom2Id);
                constructConvexPatchArc(atom1, atom2, probe1, probe2, centerProbe);
            }
            for (SphericalPatch sp : Surface.convexPatches){
                ArcUtil.linkArcs(sp);
                for (Boundary b : sp.boundaries) {
                    for (Boundary c : sp.boundaries) {
                        if (c == b) {
                            continue;
                        }
                        if (ArcUtil.checkIfNested(b, c)) {
                            b.nestedBoundaries.add(c);
                        }
                    }
                    ArcUtil.buildEdges(b, true);
                }
                //refine arcs - when refining an arc, its opposite arc will be refined as well as to have the same number of vertices on both of them
                Surface.atomsProcessed.set(sp.id + 1);
            }
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public static ArrayList<SphericalPatch> parseAtoms(String filename){
        double x, y, z, r;
        int id = -1;
        ArrayList<SphericalPatch> n = new ArrayList<>(SesConfig.atomCount);
        try (DataInputStream in = new DataInputStream(new FileInputStream(filename))){
            byte[] buffer = new byte[SesConfig.atomCount * 20];
            in.read(buffer, 0, buffer.length);
            ByteBuffer data = ByteBuffer.wrap(buffer);
            for (int i = 0; i < SesConfig.atomCount; i++){
                id = data.getInt(); //dont delete
                x = data.getFloat();
                y = data.getFloat();
                z = data.getFloat();
                r = data.getFloat();
                n.add(new SphericalPatch(new Point(x, y, z), r, true));
            }
        } catch (IOException e){
            e.printStackTrace();
        }
        return n;
    }

    public static void parseTriangles(String filename){
        double x, y, z;
        int a1, a2, a3;
        try (DataInputStream in = new DataInputStream(new FileInputStream(filename))){
            byte[] buffer = new byte[SesConfig.trianglesCount * 24];
            in.read(buffer, 0, buffer.length);
            ByteBuffer data = ByteBuffer.wrap(buffer);
            for (int i = 0; i < SesConfig.trianglesCount; ++i){
                a1 = data.getInt();
                a2 = data.getInt();
                a3 = data.getInt();
                x = data.getFloat();
                y = data.getFloat();
                z = data.getFloat();
                constructConcavePatchArcs(new Sphere(new Point(x, y, z), SesConfig.probeRadius), a1, a2, a3);
            }
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    private static void constructConvexPatchArc(SphericalPatch atom1, SphericalPatch atom2, Sphere probe1, Sphere probe2, Sphere probeMid){
        if (Point.subtractPoints(probe1.center, probe2.center).sqrtMagnitude() < 0.0001 && Point.subtractPoints(probe1.center, probeMid.center).sqrtMagnitude() > 0.01){
            //Vector a1Toa2 = Point.subtractPoints(atom2.sphere.center, atom1.sphere.center).makeUnit();
            Arc[] atom1Arcs = ArcUtil.makeNewArc(atom1, atom2, Sphere.getContactPoint(atom1.sphere, probe1), Sphere.getContactPoint(atom1.sphere, probe2), Sphere.getContactPoint(atom1.sphere, probeMid), probeMid.center, true);
            Arc[] atom2Arcs = ArcUtil.makeNewArc(atom2, atom1, Sphere.getContactPoint(atom2.sphere, probe1), Sphere.getContactPoint(atom2.sphere, probe2), Sphere.getContactPoint(atom2.sphere, probeMid), probeMid.center, true);
            for (Arc a : atom1Arcs){
                for (Arc j : atom2Arcs){
                    Vector midtomid = Point.subtractPoints(j.mid, a.mid).makeUnit();
                    //if (Math.abs(Math.abs(midtomid.dotProduct(a1Toa2)) - 1) < 0.001){
                    if (Point.distance(a.midProbe, j.midProbe) < 0.001){
                        a.opposite = j;
                        j.opposite = a;
                                /*RollingPatch rp = new RollingPatch();
                                rp.circular = true;
                                rp.midProbe = j.midProbe;
                                rp.probe1 = probe1.center;
                                rp.probe2 = probeMid.center;
                                rp.cxpl1 = a;
                                rp.cxpl2 = j;
                                rp.convexPatchArcs.add(a);
                                rp.convexPatchArcs.add(j);*/
                        ToroidalPatch tp = new ToroidalPatch(probe1.center, probeMid.center, j.midProbe);
                        tp.convexPatchArcs.add(a);
                        tp.convexPatchArcs.add(j);
                        tp.circular = true;

                        assignRollingPatchToAtoms(atom1, atom2, tp);
                        Arc smallerRadius = (a.owner.sphere.radius <= j.owner.sphere.radius) ? a : j;
                        Arc greaterRadius = (smallerRadius == a) ? j : a;
                        //ArcUtil.refineArc(smallerRadius, Main.maxEdgeLen, true,2, false);
                        ArcUtil.refineArc(smallerRadius, Surface.maxEdgeLen, false,0, false);
                        ArcUtil.buildEdges(smallerRadius);
                        //int numOfDivs = (int)(Math.log10(smallerRadius.lines.size() / 2) / Math.log10(2));
                        int numOfDivs = ArcUtil.getSubdivisionLevel(smallerRadius);
                        ArcUtil.refineArc(greaterRadius, Surface.maxEdgeLen, true, numOfDivs, false);
                        ArcUtil.buildEdges(greaterRadius);

                        smallerRadius.refined = ArcUtil.dbgCloneArc(smallerRadius);
                        greaterRadius.refined = ArcUtil.dbgCloneArc(greaterRadius);

                        smallerRadius.refined.owner = smallerRadius.owner;
                        greaterRadius.refined.owner = greaterRadius.owner;

                        ArcUtil.refineOppositeArcs(smallerRadius.refined, greaterRadius.refined, SesConfig.edgeLimit, true);

                        System.out.println("refined circle loop: " + smallerRadius.vrts.size() + ", " + greaterRadius.vrts.size());
                    }
                }
            }
            System.out.println("Constructed circular loop for: " + atom1.id + " and " + atom2.id);
            return;
        } else if (Point.subtractPoints(probe1.center, probe2.center).sqrtMagnitude() < 0.0001){
            //System.out.println("small atom id: " + atom1Id + " " + atom2Id);
        }
        //ConvexPatchArc l1 = atom1.makeNewLoop(Atom.getContactPoint(atom1, probe1), Atom.getContactPoint(atom1, probe2), Atom.getContactPoint(atom1, probeMid), atom2, probeMid.center,false)[0];
        //ConvexPatchArc l2 = atom2.makeNewLoop(Atom.getContactPoint(atom2, probe2), Atom.getContactPoint(atom2, probe1), Atom.getContactPoint(atom2, probeMid), atom1, probeMid.center,false)[0];
        Arc arc1 = ArcUtil.makeNewArc(atom1, atom2, Sphere.getContactPoint(atom1.sphere, probe1), Sphere.getContactPoint(atom1.sphere, probe2), Sphere.getContactPoint(atom1.sphere, probeMid), probeMid.center, false)[0];
        Arc arc2 = ArcUtil.makeNewArc(atom2, atom1, Sphere.getContactPoint(atom2.sphere, probe2), Sphere.getContactPoint(atom2.sphere, probe1), Sphere.getContactPoint(atom2.sphere, probeMid), probeMid.center, false)[0];
        arc1.opposite = arc2;
        arc2.opposite = arc1;
                /*RollingPatch rp = new RollingPatch();
                rp.cxpl1 = arc1;
                rp.cxpl2 = arc2;
                rp.convexPatchArcs.add(arc1);
                rp.convexPatchArcs.add(arc2);
                rp.midProbe = probeMid.center;
                rp.probe1 = probe1.center;
                rp.probe2 = probe2.center;*/
        ToroidalPatch tp = new ToroidalPatch(probe1.center, probe2.center, probeMid.center);
        tp.convexPatchArcs.add(arc1);
        tp.convexPatchArcs.add(arc2);
        assignRollingPatchToAtoms(atom1, atom2, tp);
        Arc smallerRadius = (arc1.owner.sphere.radius <= arc2.owner.sphere.radius) ? arc1 : arc2;
        Arc greaterRadius = (smallerRadius == arc1) ? arc2 : arc1;
        //ArcUtil.refineArc(smallerRadius, Main.maxEdgeLen, 2, false);
        ArcUtil.refineArc(smallerRadius, Surface.maxEdgeLen, false,0, false);
        ArcUtil.buildEdges(smallerRadius);
        //int numOfDivs = (int)(Math.log10(smallerRadius.lines.size() / 2) / Math.log10(2));
        int numOfDivs = ArcUtil.getSubdivisionLevel(smallerRadius);
        ArcUtil.refineArc(greaterRadius, Surface.maxEdgeLen, true, numOfDivs, false);
        ArcUtil.buildEdges(greaterRadius);
        smallerRadius.refined = ArcUtil.dbgCloneArc(smallerRadius);
        smallerRadius.refined.owner = smallerRadius.owner;
        greaterRadius.refined = ArcUtil.dbgCloneArc(greaterRadius);
        greaterRadius.refined.owner = greaterRadius.owner;
        ArcUtil.refineOppositeArcs(smallerRadius.refined, greaterRadius.refined, SesConfig.edgeLimit, true);
        if (smallerRadius.vrts.size() != greaterRadius.vrts.size()){
            System.err.println("inconsistency detected");
        }
        tp.width = Point.distance(Sphere.getContactPoint(atom1.sphere, probe1), Sphere.getContactPoint(atom1.sphere, probeMid)) + Point.distance(Sphere.getContactPoint(atom1.sphere, probeMid), Sphere.getContactPoint(atom1.sphere, probe2));
        if (tp.width < 0.005){
            //Surface.smallRectangles.add(tp);
            //System.err.println("added small rectangle to data structure");
        }
        if (PatchUtil.getProbeAxisDistance(probe1.center, atom1.sphere.center, atom2.sphere.center) - SesConfig.probeRadius < 0.0){
            Surface.selfIntersectingRects.add(tp);
        }
    }
    public static void assignRollingPatchToAtoms(SphericalPatch s1, SphericalPatch s2, ToroidalPatch tp){
        if (!s1.tori.containsKey(s2.id)){
            s1.tori.put(s2.id, new ArrayList<>());
        }
        if (!s2.tori.containsKey(s1.id)){
            s2.tori.put(s1.id, new ArrayList<>());
        }
        s1.tori.get(s2.id).add(tp);
        s2.tori.get(s1.id).add(tp);
        //rp.id = Main.rectangles.size();
        Surface.rectangles.add(tp);
    }

    private static void constructConcavePatchArcs(Sphere probe, int atom1, int atom2, int atom3){
        try {
            SphericalPatch a1 = Surface.convexPatches.get(atom1);
            SphericalPatch a2 = Surface.convexPatches.get(atom2);
            SphericalPatch a3 = Surface.convexPatches.get(atom3);

            Point a1touch = Sphere.getContactPoint(a1.sphere, probe);
            Point a2touch = Sphere.getContactPoint(a2.sphere, probe);
            Point a3touch = Sphere.getContactPoint(a3.sphere, probe);

            Point mid = Point.getMidPoint(a1touch, a2touch);
            Vector probeMid = Point.subtractPoints(mid, probe.center).makeUnit().multiply(probe.radius);
            mid = Point.translatePoint(probe.center, probeMid);
            SphericalPatch cpatch = new SphericalPatch(probe, false);
            Arc cpl1 = new Arc(probe.center, probe.radius);
            cpl1.vrts.add(a1touch);
            cpl1.vrts.add(mid);
            cpl1.vrts.add(a2touch);

            //cpl1.end1 = a1touch;
            //cpl1.end2 = a2touch;
                /*cpl1.toEnd1 = Point.subtractPoints(cpl1.end1, cpl1.center).makeUnit();
                cpl1.toEnd2 = Point.subtractPoints(cpl1.end2, cpl1.center).makeUnit();
                cpl1.normal = Vector.getNormalVector(cpl1.toEnd1, cpl1.toEnd2).makeUnit();*/

            ToroidalPatch tp = null;
            if (a1 == null || a2 == null || a3 == null) {
                System.out.println(" atom null");
            }
            if (a1.tori.get(atom2) == null) {
                //System.out.println("corresponding rolling patch not found for " + atom1 + " " + atom2);
                //continue;
            } else {
                for (ToroidalPatch tor : a1.tori.get(atom2)) {
                    if (Point.subtractPoints(probe.center, tor.probe1).sqrtMagnitude() < 0.005 || Point.subtractPoints(probe.center, tor.probe2).sqrtMagnitude() < 0.005) {
                        tp = tor;
                    }
                }
            }
            if (tp == null) {
                System.out.println("corresponding rolling patch not found");
                updateMissingArcs(atom1, atom2, cpatch);
            } else {
                tp.concavePatchArcs.add(cpl1);
                cpl1.torus = tp;
                /*cpl1.vrts.clear();
                Arc ar1 = (tp.convexPatchArcs.get(0).owner.id == atom1) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
                Arc ar2 = (tp.convexPatchArcs.get(0) == ar1) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);
                cpl1.vrts.add((Point.distance(ar1.end1, a1touch) < 0.0001) ? ar1.end1 : ar1.end2);
                cpl1.vrts.add(mid);
                cpl1.vrts.add((Point.distance(ar2.end1, a2touch) < 0.0001) ? ar2.end1 : ar2.end2);*/
            }
                /*Vector v1mid = Point.subtractPoints(mid, a1touch).makeUnit();
                Vector v1v2 = Point.subtractPoints(a2touch, a1touch).makeUnit();
                Vector v1probe = Point.subtractPoints(probe.center, a1touch).makeUnit();*/
            Vector v1v2 = Point.subtractPoints(a2touch, a1touch).makeUnit();
            Vector v1mid = Point.subtractPoints(a3touch, a1touch).makeUnit();
            Vector v1probe = Point.subtractPoints(probe.center, a1.sphere.center).makeUnit();
                /*if (AdvancingFrontMethod.determinant(v1v2, v1mid, v1probe) < 0){
                    Util.reverserOrder(cpl1);
                }*/
            if (VectorUtil.determinantVec3(v1v2.getFloatData(), v1mid.getFloatData(), v1probe.getFloatData()) > 0.f) {
                ArcUtil.reverseArc(cpl1, true);
            }
                /*Edge e1 = new Edge(0, 1);
                Edge e2 = new Edge(1, 2);
                e1.p1 = cpl1.vrts.get(0);
                e1.p2 = mid;
                e2.p1 = mid;
                e2.p2 = cpl1.vrts.get(2);
                e1.next = e2;
                e2.prev = e1;*/
                /*cpl1.endEdge1 = e1;
                cpl1.endEdge2 = e2;
                cpl1.end1 = cpl1.vrts.get(0);
                cpl1.end2 = cpl1.vrts.get(2);
                cpl1.lines.add(e1);
                cpl1.lines.add(e2);*/
                /*Edge edge = new Edge(0, 1);
                edge.p1 = cpl1.vrts.get(0);
                edge.p2 = cpl1.vrts.get(1);
                cpl1.endEdge1 = edge;
                cpl1.endEdge2 = edge;
                edge.prev = edge;
                edge.next = edge;
                cpl1.lines.add(edge);
                cpl1.end1 = cpl1.vrts.get(0);
                cpl1.end2 = cpl1.vrts.get(1);*/
            cpl1.end1 = cpl1.vrts.get(0);
            cpl1.end2 = cpl1.vrts.get(2);
            cpl1.mid = mid;

            cpl1.setEndPoints(cpl1.vrts.get(0), cpl1.vrts.get(2), true);

            cpl1.endEdge1 = new Edge(0, 1);
            cpl1.endEdge1.p1 = cpl1.end1;
            cpl1.endEdge1.p2 = cpl1.mid;
            cpl1.endEdge2 = new Edge(1, 2);
            cpl1.endEdge2.p1 = cpl1.mid;
            cpl1.endEdge2.p2 = cpl1.end2;
            cpl1.endEdge1.next = cpl1.endEdge2;
            cpl1.endEdge2.prev = cpl1.endEdge1;

            mid = Point.getMidPoint(a1touch, a3touch);
            probeMid = Point.subtractPoints(mid, probe.center).makeUnit().multiply(probe.radius);
            mid = Point.translatePoint(probe.center, probeMid);
            Arc cpl2 = new Arc(probe.center, probe.radius);
            cpl2.vrts.add(a3touch);
            cpl2.vrts.add(mid);
            cpl2.vrts.add(a1touch);
            //cpl2.atom = probe;

            tp = null;
            if (atom1 == 2556 && atom2 == 3077 && atom3 == 3085) {
                System.out.println(" ");
            }
            if (a1.tori.get(atom3) == null) {
                //System.out.println("corresponding rolling patch not found for" + atom1 + " " + atom3);
                //continue;
            } else {
                for (ToroidalPatch tor : a1.tori.get(atom3)) {
                    if (Point.subtractPoints(probe.center, tor.probe1).sqrtMagnitude() < Surface.scaleFactor * 0.005 || Point.subtractPoints(probe.center, tor.probe2).sqrtMagnitude() < Surface.scaleFactor * 0.005) {
                        tp = tor;
                    }
                }
            }
            if (tp == null) {
                System.out.println("corresponding rolling patch not found");
                //continue;
                updateMissingArcs(atom1, atom3, cpatch);
            } else {
                tp.concavePatchArcs.add(cpl2);
                cpl2.torus = tp;
                /*cpl2.vrts.clear();
                Arc ar1 = (tp.convexPatchArcs.get(0).owner.id == atom3) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
                Arc ar2 = (tp.convexPatchArcs.get(0) == ar1) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);
                cpl2.vrts.add((Point.distance(ar1.end1, a3touch) < 0.0001) ? ar1.end1 : ar1.end2);
                cpl2.vrts.add(mid);
                cpl2.vrts.add((Point.distance(ar2.end1, a1touch) < 0.0001) ? ar2.end1 : ar2.end2);*/
            }
                /*v1mid = Point.subtractPoints(mid, a1touch).makeUnit();
                v1v2 = Point.subtractPoints(a3touch, a1touch).makeUnit();*/
            v1v2 = Point.subtractPoints(a1touch, a3touch).makeUnit();
            v1mid = Point.subtractPoints(a2touch, a3touch).makeUnit();
            v1probe = Point.subtractPoints(probe.center, a3.sphere.center).makeUnit();
                /*if (AdvancingFrontMethod.determinant(v1v2, v1mid, v1probe) < 0){
                    Util.reverserOrder(cpl2);
                }*/
            if (VectorUtil.determinantVec3(v1v2.getFloatData(), v1mid.getFloatData(), v1probe.getFloatData()) > 0.f) {
                ArcUtil.reverseArc(cpl2, true);
            }
                /*e1 = new Edge(0, 1);
                e2 = new Edge(1, 2);
                e1.p1 = cpl2.vrts.get(0);
                e1.p2 = mid;
                e2.p1 = mid;
                e2.p2 = cpl2.vrts.get(2);
                e1.next = e2;
                e2.prev = e1;*/
                /*cpl2.endEdge1 = e1;
                cpl2.endEdge2 = e2;
                cpl2.end1 = cpl2.vrts.get(0);
                cpl2.end2 = cpl2.vrts.get(2);
                cpl2.lines.add(e1);
                cpl2.lines.add(e2);*/
                /*edge = new Edge(0, 1);
                edge.p1 = cpl2.vrts.get(0);
                edge.p2 = cpl2.vrts.get(1);
                edge.next = edge;
                edge.prev = edge;
                cpl2.endEdge2 = edge;
                cpl2.endEdge1 = edge;
                cpl2.lines.add(edge);
                cpl2.end1 = cpl2.vrts.get(0);
                cpl2.end2 = cpl2.vrts.get(1);*/

            //cpl2.end1 = cpl2.vrts.get(0);
            //cpl2.end2 = cpl2.vrts.get(2);
            cpl2.mid = mid;

            //cpl2.toEnd1 = Point.subtractPoints(cpl2.end1, cpl2.center).makeUnit();
            //cpl2.toEnd2 = Point.subtractPoints(cpl2.end2, cpl2.center).makeUnit();
            //cpl2.normal = Vector.getNormalVector(cpl2.toEnd1, cpl2.toEnd2).makeUnit();
            cpl2.setEndPoints(cpl2.vrts.get(0), cpl2.vrts.get(2), true);

            cpl2.endEdge1 = new Edge(0, 1);
            cpl2.endEdge1.p1 = cpl2.end1;
            cpl2.endEdge1.p2 = cpl2.mid;
            cpl2.endEdge2 = new Edge(1, 2);
            cpl2.endEdge2.p1 = cpl2.mid;
            cpl2.endEdge2.p2 = cpl2.end2;
            cpl2.endEdge1.next = cpl2.endEdge2;
            cpl2.endEdge2.prev = cpl2.endEdge1;

            mid = Point.getMidPoint(a2touch, a3touch);
            probeMid = Point.subtractPoints(mid, probe.center).makeUnit().multiply(probe.radius);
            mid = Point.translatePoint(probe.center, probeMid);
            Arc cpl3 = new Arc(probe.center, probe.radius);
            cpl3.vrts.add(a2touch);
            cpl3.vrts.add(mid);
            cpl3.vrts.add(a3touch);
            //cpl3.atom = probe;

            tp = null;
            if (a2.tori.get(atom3) == null) {
               System.out.println("corresponding rolling patch not found for " + atom2 + " " + atom3);
                //continue;
                if (atom1 == 224 || atom2 == 224 || atom3 == 224){
                    System.out.println("for atom 224");
                }
            } else {
                for (ToroidalPatch tor : a2.tori.get(atom3)) {
                    if (Point.subtractPoints(probe.center, tor.probe1).sqrtMagnitude() < Surface.scaleFactor * 0.005 || Point.subtractPoints(probe.center, tor.probe2).sqrtMagnitude() < Surface.scaleFactor * 0.005) {
                        tp = tor;
                    }
                }
            }
            if (tp == null) {
                //System.out.println("corresponding rolling patch not found");
                //continue;
                updateMissingArcs(atom2, atom3, cpatch);
            } else {
                tp.concavePatchArcs.add(cpl3);
                cpl3.torus = tp;
                /*cpl3.vrts.clear();
                Arc ar1 = (tp.convexPatchArcs.get(0).owner.id == atom2) ? tp.convexPatchArcs.get(0) : tp.convexPatchArcs.get(1);
                Arc ar2 = (tp.convexPatchArcs.get(0) == ar1) ? tp.convexPatchArcs.get(1) : tp.convexPatchArcs.get(0);
                cpl3.vrts.add((Point.distance(ar1.end1, a2touch) < 0.0001) ? ar1.end1 : ar1.end2);
                cpl3.vrts.add(mid);
                cpl3.vrts.add((Point.distance(ar2.end1, a3touch) < 0.0001) ? ar2.end1 : ar2.end2);*/
            }
                /*v1v2 = Point.subtractPoints(a3touch, a2touch).makeUnit();
                v1mid = Point.subtractPoints(mid, a2touch).makeUnit();
                v1probe = Point.subtractPoints(probe.center, a2touch).makeUnit();*/
            v1v2 = Point.subtractPoints(a3touch, a2touch).makeUnit();
            v1mid = Point.subtractPoints(a1touch, a2touch).makeUnit();
            v1probe = Point.subtractPoints(probe.center, a2.sphere.center).makeUnit();
                /*if (AdvancingFrontMethod.determinant(v1v2, v1mid, v1probe) < 0){
                    Util.reverserOrder(cpl3);
                }*/
            if (VectorUtil.determinantVec3(v1v2.getFloatData(), v1mid.getFloatData(), v1probe.getFloatData()) > 0.f) {
                ArcUtil.reverseArc(cpl3, true);
            }
                /*e1 = new Edge(0, 1);
                e2 = new Edge(1, 2);
                e1.p1 = cpl3.vrts.get(0);
                e1.p2 = mid;
                e2.p1 = mid;
                e2.p2 = cpl3.vrts.get(2);
                e1.next = e2;
                e2.prev = e1;*/
                /*cpl3.endEdge1 = e1;
                cpl3.endEdge2 = e2;
                cpl3.end1 = cpl3.vrts.get(0);
                cpl3.end2 = cpl3.vrts.get(2);
                cpl3.lines.add(e1);
                cpl3.lines.add(e2);*/
                /*edge = new Edge(0, 1);
                edge.p1 = cpl3.vrts.get(0);
                edge.p2 = cpl3.vrts.get(1);
                edge.next = edge;
                edge.prev = edge;
                cpl3.endEdge1 = edge;
                cpl3.endEdge2 = edge;
                cpl3.lines.add(edge);
                cpl3.end1 = cpl3.vrts.get(0);
                cpl3.end2 = cpl3.vrts.get(1);*/

            //cpl3.end1 = cpl3.vrts.get(0);
            //cpl3.end2 = cpl3.vrts.get(2);
            cpl3.mid = mid;

                /*cpl3.toEnd1 = Point.subtractPoints(cpl3.end1, cpl3.center).makeUnit();
                cpl3.toEnd2 = Point.subtractPoints(cpl3.end2, cpl3.center).makeUnit();
                cpl3.normal = Vector.getNormalVector(cpl3.toEnd1, cpl3.toEnd2).makeUnit();*/
            cpl3.setEndPoints(cpl3.vrts.get(0), cpl3.vrts.get(2), true);

            cpl3.endEdge1 = new Edge(0, 1);
            cpl3.endEdge1.p1 = cpl3.end1;
            cpl3.endEdge1.p2 = cpl3.mid;
            cpl3.endEdge2 = new Edge(1, 2);
            cpl3.endEdge2.p1 = cpl3.mid;
            cpl3.endEdge2.p2 = cpl3.end2;
            cpl3.endEdge1.next = cpl3.endEdge2;
            cpl3.endEdge2.prev = cpl3.endEdge1;

            //Point end = cpl1.vrts.get(2);
            List<Arc> q = new ArrayList<>();
            //q.add(cpl1);
            q.add(cpl2);
            q.add(cpl3);
            Point start = cpl1.end1;
            Point pivot = cpl1.end2;
            Arc pivotLoop = cpl1;
            int i = 0;
            boolean ghost = false;
            do {
                if (q.size() == 0){
                    System.out.println("");
                }
                Arc l = q.get(i);
                if (Point.subtractPoints(pivot, l.end1).sqrtMagnitude() < 0.0001) {
                    pivotLoop.next = l;
                    l.prev = pivotLoop;
                    pivotLoop.endEdge2.next = l.endEdge1;
                    l.endEdge1.prev = pivotLoop.endEdge2;
                    pivot = l.end2;
                    pivotLoop = l;
                    q.remove(l);
                    i = 0;
                } else {
                    i++;
                    if (i >= q.size()) {
                        ghost = true;
                        break;
                    }
                }
            } while (Point.subtractPoints(start, pivot).sqrtMagnitude() >= 0.0001);

            if (ghost) {
                /*if (cpl1.torus.id == 0) {
                    System.out.println("fdsa");
                }*/
                cpl1.owner = cpatch;
                cpl2.owner = cpatch;
                cpl3.owner = cpatch;
                cpl1.valid = cpl2.valid = cpl3.valid = false;
                return;
            }
            cpl1.prev = pivotLoop;
            pivotLoop.next = cpl1;
            pivotLoop.endEdge2.next = cpl1.endEdge1;
            cpl1.endEdge1.prev = pivotLoop.endEdge2;
            Boundary b = new Boundary();
            //b.arcs = new ArrayList<>();
            //b.vrts = new ArrayList<>();
            //b.lines = new ArrayList<>();
            b.arcs.add(cpl1);
            b.arcs.add(cpl1.next);
            b.arcs.add(cpl1.prev);
            cpl1.bOwner = cpl2.bOwner = cpl3.bOwner = b;


            cpatch.boundaries.add(b);
            b.patch = cpatch;

            //ConcavePatch cpatch = new ConcavePatch(b);
            //cpatch.probe = probe.center;
            //cpatch.id = Main.triangles.size();
            //cpatch.probe2 = probe;

            cpl1.owner = cpatch;
            cpl2.owner = cpatch;
            cpl3.owner = cpatch;
            ArcUtil.refineArc(cpl1, Surface.maxEdgeLen, false, 0, false);
            ArcUtil.refineArc(cpl2, Surface.maxEdgeLen, false, 0, false);
            ArcUtil.refineArc(cpl3, Surface.maxEdgeLen, false, 0, false);
            cpl1.valid = cpl2.valid = cpl3.valid = true;
            ArcUtil.buildEdges(b, true);
            //cpl1.circularLoop = false;
            //cpl2.circularLoop = false;
            //cpl3.circularLoop = false;
                /*cpatch.atomIDs[0] = atom1;
                cpatch.atomIDs[1] = atom2;
                cpatch.atomIDs[2] = atom3;*/
            Vector u = Point.subtractPoints(cpl1.next.end2, cpl1.end2).makeUnit();
            Vector v = Point.subtractPoints(cpl1.end1, cpl1.end2).makeUnit();
            //cpatch.plane = new Plane(cpl1.end1, Vector.getNormalVector(u, v).makeUnit());
            //cpatch.planeNormal = Vector.getNormalVector(u, v).makeUnit();
            Surface.triangles.add(cpatch);
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static void jsonParseTriangles(String json){
        int atom1 = -1, atom2 = -1, atom3 = -1;
        //Point ea1 = null, ea2 = null, ea3 = null;
        //boolean found = false;
        try {
            JSONParser parser = new JSONParser();
            JSONArray jArray = (JSONArray)parser.parse(json);
            int count = 0;
            for (Object obj : jArray) {
                count++;
                JSONObject jObj = (JSONObject) obj;
                atom1 = ((Long)jObj.get("atom1Id")).intValue();
                atom2 = ((Long)jObj.get("atom2Id")).intValue();
                atom3 = ((Long)jObj.get("atom3Id")).intValue();
                JSONObject jProbe = (JSONObject) jObj.get("sphere");
                //Atom probe = new Atom(new Point((double) jProbe.get("x"), (double) jProbe.get("y"), (double) jProbe.get("z")), (double) jProbe.get("r"));
                //Atom probe = new Atom(new Point(jProbe), Main.scaleFactor * (double)jProbe.get("r"));
                Sphere probe = new Sphere(new Point(jProbe), SesConfig.probeRadius);
                constructConcavePatchArcs(probe, atom1, atom2, atom3);
            }
            System.out.println(count + " triangles loaded");
        } catch (Exception e){
            System.err.println("a1: " + atom1 + " a2: " + atom2 + " a3: " + atom3);
            e.printStackTrace();
        }
    }

    public static boolean exportOBJ(String filename, char mask){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(filename))){
            int offset = 0;
            int ownIdx = Surface.commonVrts.size() + 1;
            for (Point p : Surface.commonVrts){
                bw.write("v " + p.toString());
                bw.newLine();
            }
            for (Vector v : Surface.normals){
                bw.write("vn " + v.toString());
                bw.newLine();
            }
            if ((mask & 1) > 0) {
                for (SphericalPatch a : Surface.convexPatches) {
                    List<Point> vrts = a.vertices;
                    List<Face> faces = a.faces;
                /*for (int i = 0; i < vrts.size(); ++i){

                }*/
                    List<Vector> norms = new ArrayList<>();
                    int ownVerticesCount = 0;
                    for (Point p : vrts) {
                        if (p.idx > 0) {
                            continue;
                        }
                        Vector n = Point.subtractPoints(p, a.sphere.center).makeUnit();
                        norms.add(n);
                        bw.write("v " + p.toString());
                        bw.newLine();
                        bw.write("vn " + n.toString());
                        bw.newLine();
                        ownVerticesCount++;
                        p.ownIdx = ownIdx++;
                    }

                    for (Face f : faces) {
                        Point p = vrts.get(f.a);
                        Point q = vrts.get(f.b);
                        Point r = vrts.get(f.c);
                        String line = "f ";
                        if (p.idx > 0) {
                            line += Integer.toString(p.idx) + "//" + Integer.toString(p.idx);
                        } else {
                            line += Integer.toString(p.ownIdx) + "//" + Integer.toString(p.ownIdx);
                            //line += Integer.toString(f.a + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.a + Main.commonVrts.size() + offset);
                        }
                        line += " ";
                        if (q.idx > 0) {
                            line += Integer.toString(q.idx) + "//" + Integer.toString(q.idx);
                        } else {
                            line += Integer.toString(q.ownIdx) + "//" + Integer.toString(q.ownIdx);
                            //line += Integer.toString(f.b + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.b + Main.commonVrts.size() + offset);
                        }
                        line += " ";
                        if (r.idx > 0) {
                            line += Integer.toString(r.idx) + "//" + Integer.toString(r.idx);
                        } else {
                            line += Integer.toString(r.ownIdx) + "//" + Integer.toString(r.ownIdx);
                            //line += Integer.toString(f.c + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.c + Main.commonVrts.size() + offset);
                        }
                        bw.write(line);
                        bw.newLine();
                    }
                    offset += ownVerticesCount;
                }
            }
            if ((mask & 2) > 0) {
                for (SphericalPatch cp : Surface.triangles) {
                    List<Point> vrts = cp.vertices;
                    List<Face> faces = cp.faces;
                    int ownVerticesCount = 0;
                    for (Point p : vrts) {
                        if (p.idx > 0) {
                            continue;
                        }
                        Vector n = Point.subtractPoints(cp.sphere.center, p).makeUnit();
                        bw.write("v " + p.toString());
                        bw.newLine();
                        bw.write("vn " + n.toString());
                        bw.newLine();
                        ownVerticesCount++;
                        p.ownIdx = ownIdx++;
                    }
                    for (Face f : faces) {
                        Point p = vrts.get(f.a);
                        Point q = vrts.get(f.b);
                        Point r = vrts.get(f.c);
                        String line = "f ";
                        if (r.idx > 0) {
                            line += Integer.toString(r.idx) + "//" + Integer.toString(r.idx);
                        } else {
                            line += Integer.toString(r.ownIdx) + "//" + Integer.toString(r.ownIdx);
                            //line += Integer.toString(f.c + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.c + Main.commonVrts.size() + offset);
                        }
                        line += " ";
                        if (q.idx > 0) {
                            line += Integer.toString(q.idx) + "//" + Integer.toString(q.idx);
                        } else {
                            line += Integer.toString(q.ownIdx) + "//" + Integer.toString(q.ownIdx);
                            //line += Integer.toString(f.b + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.b + Main.commonVrts.size() + offset);
                        }
                        line += " ";
                        if (p.idx > 0) {
                            line += Integer.toString(p.idx) + "//" + Integer.toString(p.idx);
                        } else {
                            line += Integer.toString(p.ownIdx) + "//" + Integer.toString(p.ownIdx);
                            //line += Integer.toString(f.a + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.a + Main.commonVrts.size() + offset);
                        }
                        bw.write(line);
                        bw.newLine();
                    }
                    offset += ownVerticesCount;
                }
            }
            if ((mask & 4) > 0) {
                for (ToroidalPatch tp : Surface.rectangles) {
                    List<Point> vrts = tp.vertices;
                    List<Vector> normals = tp.normals;
                    List<Face> faces = tp.faces;
                    int ownVerticesCount = 0;
                    for (int i = 0; i < vrts.size(); ++i) {
                        Point p = vrts.get(i);
                        if (p.idx > 0) {
                            continue;
                        }
                        bw.write("v " + p.toString());
                        bw.newLine();
                        bw.write("vn " + normals.get(i).toString());
                        bw.newLine();
                        ownVerticesCount++;
                        p.ownIdx = ownIdx++;
                    }
                    for (Face f : faces) {
                        Point p = vrts.get(f.a);
                        Point q = vrts.get(f.b);
                        Point r = vrts.get(f.c);
                        String line = "f ";
                        if (p.idx > 0) {
                            line += Integer.toString(p.idx) + "//" + Integer.toString(p.idx);
                        } else {
                            line += Integer.toString(p.ownIdx) + "//" + Integer.toString(p.ownIdx);
                            //line += Integer.toString(f.a + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.a + Main.commonVrts.size() + offset);
                        }
                        line += " ";
                        if (q.idx > 0) {
                            line += Integer.toString(q.idx) + "//" + Integer.toString(q.idx);
                        } else {
                            line += Integer.toString(q.ownIdx) + "//" + Integer.toString(q.ownIdx);
                            //line += Integer.toString(f.b + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.b + Main.commonVrts.size() + offset);
                        }
                        line += " ";
                        if (r.idx > 0) {
                            line += Integer.toString(r.idx) + "//" + Integer.toString(r.idx);
                        } else {
                            line += Integer.toString(r.ownIdx) + "//" + Integer.toString(r.ownIdx);
                            //line += Integer.toString(f.c + Main.commonVrts.size() + offset) + "//" + Integer.toString(f.c + Main.commonVrts.size() + offset);
                        }
                        bw.write(line);
                        bw.newLine();
                    }
                    offset += ownVerticesCount;
                }
            }
            bw.flush();
            bw.close();
        } catch (IOException e){
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public static boolean exportSTL(String file){
        try (DataOutputStream ds = new DataOutputStream(new FileOutputStream(file))){
            /*for (int i = 0; i < 20; ++i){
                ds.writeInt(i);
            }*/
            for (int i = 0; i < 80; ++i){
                ds.writeByte(0);
            }
            ds.writeInt(Surface.numoftriangles);
            List<Point> vrts;
            List<Vector> normals;
            List<Face> faces;
            for (SphericalPatch a : Surface.convexPatches){
                vrts = a.vertices;
                faces = a.faces;
                for (Face f : faces) {
                    ds.writeFloat(0.f);
                    ds.writeFloat(0.f);
                    ds.writeFloat(0.f);

                    Point p = vrts.get(f.c);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    p = vrts.get(f.b);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    p = vrts.get(f.a);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    ds.writeShort(0);
                }
            }
            for (SphericalPatch cp : Surface.triangles){
                vrts = cp.vertices;
                faces = cp.faces;
                for (Face f : faces) {
                    ds.writeFloat(0.f);
                    ds.writeFloat(0.f);
                    ds.writeFloat(0.f);

                    Point p = vrts.get(f.a);
                    ds.writeFloat((float)p.x + (float)0.f);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    p = vrts.get(f.b);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    p = vrts.get(f.c);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    ds.writeShort(0);
                }
            }
            for (ToroidalPatch rp : Surface.rectangles){
                vrts = rp.vertices;
                faces = rp.faces;
                for (Face f : faces) {
                    ds.writeFloat(0.f);
                    ds.writeFloat(0.f);
                    ds.writeFloat(0.f);

                    Point p = vrts.get(f.c);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    p = vrts.get(f.b);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    p = vrts.get(f.a);
                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
                    ds.writeShort(0);
                }
            }
            ds.flush();
            ds.close();
        } catch (IOException e){
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public static boolean exportSTLText(String file){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(file))){
            List<Point> vrts;
            List<Face> faces;
            bw.write("solid ");
            bw.newLine();
            for (SphericalPatch a : Surface.convexPatches){
                vrts = a.vertices;
                faces = a.faces;
                for (Face f : faces){
                    bw.write("facet normal 0.0 0.0 0.0");
                    bw.newLine();
                    bw.write("outer loop");
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.a).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.b).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.c).toString());
                    bw.newLine();
                    bw.write("endloop");
                    bw.newLine();
                    bw.write("endfacet");
                    bw.newLine();
                }
            }
            for (SphericalPatch cp : Surface.triangles){
                vrts = cp.vertices;
                faces = cp.faces;
                for (Face f : faces){
                    bw.write("facet normal 0.0 0.0 0.0");
                    bw.newLine();
                    bw.write("outer loop");
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.c).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.b).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.a).toString());
                    bw.newLine();
                    bw.write("endloop");
                    bw.newLine();
                    bw.write("endfacet");
                    bw.newLine();
                }
            }
            for (ToroidalPatch rp : Surface.rectangles){
                vrts = rp.vertices;
                faces = rp.faces;
                for (Face f : faces){
                    bw.write("facet normal 0.0 0.0 0.0");
                    bw.newLine();
                    bw.write("outer loop");
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.a).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.b).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(f.c).toString());
                    bw.newLine();
                    bw.write("endloop");
                    bw.newLine();
                    bw.write("endfacet");
                    bw.newLine();
                }
            }
            bw.write("endsolid");
            bw.flush();
            bw.close();
        } catch (IOException e){
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public static void exportEdgesIntersection(Edge h, Edge g, Edge i, String f){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(f))){
            bw.write("v " + h.p1.toString());
            bw.newLine();
            bw.write("v " + h.p2.toString());
            bw.newLine();
            bw.write("v " + g.p1.toString());
            bw.newLine();
            bw.write("v " + g.p2.toString());
            bw.newLine();
            bw.write("v " + i.p1.toString());
            bw.newLine();
            bw.write("v " + i.p2.toString());
            bw.newLine();
            bw.write("l 1 2");
            bw.newLine();
            bw.write("l 3 4");
            bw.newLine();
            bw.write("l 5 6");
            bw.flush();
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public static void exportArcs(Arc l, Arc r, String f){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(f))){
            for (Point v : l.vrts){
                bw.write("v " + v.toString());
                bw.newLine();
            }
            for (Point v : r.vrts){
                bw.write("v " + v.toString());
                bw.newLine();
            }
            int idx = 1;
            for (int i = 1; i < l.vrts.size() + r.vrts.size(); ++i){
                if (i == l.vrts.size()){
                    continue;
                }
                bw.write("l " + i + " " + (i + 1));
                bw.newLine();
            }
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public static void exportCP(SphericalPatch cp, String f){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(f))){
            int off = 0;
            for (Boundary b : cp.boundaries){
                for (Point v : b.vrts){
                    bw.write("v " + v.toString());
                    bw.newLine();
                }
                for (int i = 1; i <= b.vrts.size(); ++i){
                    if (i == b.vrts.size()){
                        bw.write("l " + (i + off) + " " + (1 + off));
                    } else {
                        bw.write("l " + (i + off) + " " + (i + 1 + off));
                    }
                    bw.newLine();
                }
                off += b.vrts.size();
            }
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public static void exportBoundary(Boundary b, String f){
        SphericalPatch cp = new SphericalPatch(b);
        exportCP(cp, f);
    }

    public static void exportPoints(List<Point> p, Vector n, String file){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(file))){
            String s = "";
            int i = 1;
            for (Point v : p){
                s = "";
                s += "v " + v.toString();
                bw.write(s);
                s = "";
                bw.newLine();
                s += "v " + Point.translatePoint(v, n).toString();
                bw.write(s);
                bw.newLine();
                s = "";
                s += "l " + i++ + " " + i++;
                bw.write(s);
                bw.newLine();
            }
            bw.flush();
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public static void exportArcOrientation(Arc a, String file){
        List<Point> ps = new ArrayList<>();
        ps.add(a.end1);
        ps.add(a.end2);
        exportPoints(ps, a.normal, file);
    }

    public static void exportCircle(Plane plane, double r, Point p, String f){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(f))){
            Vector v = Point.subtractPoints(p, plane.p);
            Quaternion q = new Quaternion();
            q.setFromAngleNormalAxis((float)Math.toRadians(45), plane.v.getFloatData());
            float[] invec = v.getFloatData();
            double a = 0;
            String s = "";
            for (int i = 0; i <= 360 / 45; ++i){
                Point point = Point.translatePoint(plane.p, new Vector(invec));
                s += "v " + point.toString();
                bw.write(s);
                bw.newLine();
                s = "";
                q.rotateVector(invec, 0, invec, 0);
            }
            s = "";
            for (int i = 0; i < 360 / 45; ++i){
                s += "l " + (i + 1) + " " + (i + 2);
                bw.write(s);
                bw.newLine();
                s = "";
            }
            bw.flush();
        } catch (IOException e){
            e.printStackTrace();
        }
    }
}
