package cz.fi.muni.xmraz3;

import com.jogamp.opengl.math.Quaternion;
import com.jogamp.opengl.math.VectorUtil;
import cz.fi.muni.xmraz3.gui.MainWindow;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import smile.neighbor.KDTree;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.file.Paths;
import java.util.*;

/**
 * Created by radoslav on 27.2.2017.
 * Class that handles loading, parsing of the analytic representation of the SES. Also handles exporting of the surface mesh.
 */
public class SurfaceParser {

    //these are used throughout the construction of circular arcs of spherical patches
    private static Vector probeMid = new Vector(0, 0, 0);
    private static Vector v1 = new Vector(0, 0, 0);
    private static Vector v2 = new Vector(0, 0, 0);
    private static Vector v3 = new Vector(0, 0, 0);


    //------------methods for parsing of input files-------------
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
            SesConfig.probeRadius = (double)obj.get("ProbeRadius");
            SesConfig.atomCount = ((Long)obj.get("AtomsCount")).intValue();
            SesConfig.toriCount = ((Long)obj.get("TorusCount")).intValue();
            SesConfig.trianglesCount = ((Long)obj.get("TrianglesCount")).intValue();
        } catch (ParseException e){
            e.printStackTrace();
        }
    }

    public static List<SphericalPatch> parseAtomsJSON(String raw){
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
                atList.add(spatch);
            }
        } catch (ParseException e){
            System.err.println("atom parsing error");
        }
        return atList;
    }

    private static ArrayList<SphericalPatch> parseAtomsBinary(String filename){
        //System.out.println("ATOMS: " + filename);
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

    private static void parseConvexAndToriPatchesBinary(String filename){
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
                constructConvexPatchArcs(atom1, atom2, probe1, probe2, centerProbe);
            }
        } catch (IOException e){
            e.printStackTrace();
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static void parseConvexAndToriPatchesJSON(String raw){
        try{
            JSONParser parser = new JSONParser();
            Object obj = parser.parse(raw);
            JSONArray ar = (JSONArray)obj;
            for (Object oj : ar){
                JSONObject entry = (JSONObject)oj;
                int atom1Id = ((Long)entry.get("atom1Id")).intValue();
                int atom2Id = ((Long)entry.get("atom2Id")).intValue();
                SphericalPatch atom1 = Surface.convexPatches.get(atom1Id);
                SphericalPatch atom2 = Surface.convexPatches.get(atom2Id);
                JSONObject jProbe1 = (JSONObject)entry.get("probe1");
                JSONObject jProbe2 = (JSONObject)entry.get("probe2");
                JSONObject jProbeMid = (JSONObject)entry.get("center");
                Sphere probe1 = new Sphere(new Point(jProbe1), SesConfig.probeRadius);
                Sphere probe2 = new Sphere(new Point(jProbe2), SesConfig.probeRadius);
                Sphere probeMid = new Sphere(new Point(jProbeMid), SesConfig.probeRadius);

                constructConvexPatchArcs(atom1, atom2, probe1, probe2, probeMid);

                rollingCount += 2;
            }
        } catch (ParseException e){
            System.err.println(e.getMessage());
        }
    }

    private static void parseTrianglesBinary(String filename){
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

    public static void parseTrianglesJSON(String json){
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


    //----------construction of circular arcs of convex and concave patches-------------
    private static void constructConvexPatchArcs(SphericalPatch atom1, SphericalPatch atom2, Sphere probe1, Sphere probe2, Sphere probeMid){
        if (Point.distance(probe1.center, probe2.center) < 0.0001 && Point.distance(probe1.center, probeMid.center) > 0.01){
            Arc[] atom1Arcs = ArcUtil.makeNewArc(atom1, atom2, Sphere.getContactPoint(atom1.sphere, probe1), Sphere.getContactPoint(atom1.sphere, probe2), Sphere.getContactPoint(atom1.sphere, probeMid), probeMid.center, true);
            Arc[] atom2Arcs = ArcUtil.makeNewArc(atom2, atom1, Sphere.getContactPoint(atom2.sphere, probe1), Sphere.getContactPoint(atom2.sphere, probe2), Sphere.getContactPoint(atom2.sphere, probeMid), probeMid.center, true);
            if (2 * Math.PI * atom1Arcs[0].radius < 0.2) {
                atom1.arcs.remove(atom1Arcs[0]);
                atom1.arcs.remove(atom1Arcs[1]);
                atom2.arcs.remove(atom2Arcs[0]);
                atom2.arcs.remove(atom2Arcs[1]);
            }
            for (int i = 0; i < atom1Arcs.length; ++i){
                Arc a = atom1Arcs[i];
                for (int j = 0; j < atom2Arcs.length; ++j){
                    Arc b = atom2Arcs[j];
                    if (Point.distance(a.midProbe, b.midProbe) < 0.001){
                        a.opposite = b;
                        b.opposite = a;
                        ToroidalPatch tp = new ToroidalPatch(probe1.center, probeMid.center, b.midProbe);
                        tp.convexPatchArcs.add(a);
                        tp.convexPatchArcs.add(b);
                        tp.circular = true;

                        assignRollingPatchToAtoms(atom1, atom2, tp);
                        Arc smallerRadius = (a.owner.sphere.radius <= b.owner.sphere.radius) ? a : b;
                        Arc greaterRadius = (smallerRadius == a) ? b : a;
                        ArcUtil.refineArc(greaterRadius, 0, true,1, false);
                        greaterRadius.baseSubdivision = -1;
                        ArcUtil.refineArc(greaterRadius, Surface.maxEdgeLen, false,0, false);
                        //ArcUtil.buildEdges(greaterRadius);
                        int numOfDivs = ArcUtil.getSubdivisionLevel(greaterRadius);
                        ArcUtil.refineArc(smallerRadius, Surface.maxEdgeLen, true, numOfDivs, false);
                        //ArcUtil.buildEdges(smallerRadius);
                    }
                }
            }
            return;
        }
        Arc arc1 = ArcUtil.makeNewArc(atom1, atom2, Sphere.getContactPoint(atom1.sphere, probe1), Sphere.getContactPoint(atom1.sphere, probe2), Sphere.getContactPoint(atom1.sphere, probeMid), probeMid.center, false)[0];
        Arc arc2 = ArcUtil.makeNewArc(atom2, atom1, Sphere.getContactPoint(atom2.sphere, probe2), Sphere.getContactPoint(atom2.sphere, probe1), Sphere.getContactPoint(atom2.sphere, probeMid), probeMid.center, false)[0];
        arc1.opposite = arc2;
        arc2.opposite = arc1;
        ToroidalPatch tp = new ToroidalPatch(probe1.center, probe2.center, probeMid.center);
        tp.convexPatchArcs.add(arc1);
        tp.convexPatchArcs.add(arc2);
        assignRollingPatchToAtoms(atom1, atom2, tp);
        Arc smallerRadius = (arc1.owner.sphere.radius <= arc2.owner.sphere.radius) ? arc1 : arc2;
        Arc greaterRadius = (smallerRadius == arc1) ? arc2 : arc1;
        ArcUtil.refineArc(greaterRadius, Surface.maxEdgeLen, false,0, false);
        //ArcUtil.buildEdges(greaterRadius);
        int numOfDivs = ArcUtil.getSubdivisionLevel(greaterRadius);
        ArcUtil.refineArc(smallerRadius, Surface.maxEdgeLen, true, numOfDivs, false);
        //ArcUtil.buildEdges(smallerRadius);
        if (smallerRadius.vrts.size() != greaterRadius.vrts.size()){
            if (SesConfig.verbose) {
                System.err.println("inconsistency detected in: smallerRadius.vrts != greaterRadius.vrts");
            }
        }
        //detect selfintersecting torus and add it to a list for later treatment
        if (PatchUtil.getProbeAxisDistance(probe1.center, atom1.sphere.center, atom2.sphere.center) - SesConfig.probeRadius < 0.0){
            Surface.selfIntersectingRects.add(tp);
        }
    }

    private static Plane _plane = new Plane(new Point(0, 0, 0), new Vector(0, 0, 0));
    private static List<Arc> q = new ArrayList<>();
    private static void constructConcavePatchArcs(Sphere probe, int atom1, int atom2, int atom3){
        try {
            SphericalPatch a1 = Surface.convexPatches.get(atom1);
            SphericalPatch a2 = Surface.convexPatches.get(atom2);
            SphericalPatch a3 = Surface.convexPatches.get(atom3);

            Point a1touch = Sphere.getContactPoint(a1.sphere, probe);
            Point a2touch = Sphere.getContactPoint(a2.sphere, probe);
            Point a3touch = Sphere.getContactPoint(a3.sphere, probe);

            Point mid = Point.getMidPoint(a1touch, a2touch);
            probeMid.changeVector(mid, probe.center).makeUnit().multiply(probe.radius);
            mid.assignTranslation(probe.center, probeMid);

            //Vector _v1 = Point.subtractPoints(a2.sphere.center, a1.sphere.center).makeUnit();
            //Vector _v2 = Point.subtractPoints(a3.sphere.center, a1.sphere.center).makeUnit();
            v1.changeVector(a2.sphere.center, a1.sphere.center).makeUnit();
            v2.changeVector(a3.sphere.center, a1.sphere.center).makeUnit();
            Vector _n = Vector.getNormalVector(v1, v2).makeUnit();
            //Plane _plane = new Plane(a1.sphere.center, _n);
            _plane.redefine(a1.sphere.center, _n);
            if (_plane.checkPointLocation(probe.center) < 0.0){
                _n.multiply(-1.0);
            }

            SphericalPatch cpatch = new SphericalPatch(probe, false);
            cpatch.patchNormal = _n;
            Arc cpl1 = new Arc(probe.center, probe.radius);
            cpl1.vrts.add(a1touch);
            cpl1.vrts.add(mid);
            cpl1.vrts.add(a2touch);

            ToroidalPatch tp = null;
            /*if (a1 == null || a2 == null || a3 == null) { //should not happen
                if (SesConfig.verbose) {
                    System.out.println("One or more atoms of concave patch are null");
                }
            }*/
            if (a1.tori.get(atom2) == null) {
                //System.out.println("corresponding rolling patch not found for " + atom1 + " " + atom2);
                //continue;
            } else {
                for (ToroidalPatch tor : a1.tori.get(atom2)) {
                    if (Point.distance(probe.center, tor.probe1) < 0.005 || Point.distance(probe.center, tor.probe2) < 0.005){
                        tp = tor;
                    }
                }
            }
            if (tp == null) {
                if (SesConfig.verbose) {
                    System.out.println("corresponding rolling patch not found for " + atom1 + " " + atom2);
                }
            } else {
                tp.concavePatchArcs.add(cpl1);
                cpl1.torus = tp;
                if (tp.concavePatchArcs.size() == 2){
                    tp.concavePatchArcs.get(0).opposite = tp.concavePatchArcs.get(1);
                    tp.concavePatchArcs.get(1).opposite = tp.concavePatchArcs.get(0);
                }
            }

            v1.changeVector(a2touch, a1touch).makeUnit();//v1v2
            v2.changeVector(a3touch, a1touch).makeUnit();//v1mid
            v3.changeVector(probe.center, a1.sphere.center).makeUnit();//v1probe
            if (VectorUtil.determinantVec3(v1.getFloatData(), v2.getFloatData(), v3.getFloatData()) > 0.f) {
                ArcUtil.reverseArc(cpl1, true);
            }
            cpl1.end1 = cpl1.vrts.get(0);
            cpl1.end2 = cpl1.vrts.get(2);
            //cpl1.mid = mid;

            cpl1.setEndPoints(cpl1.vrts.get(0), cpl1.vrts.get(2), true);

            //cpl1.endEdge1 = new Edge(0, 1);
            //cpl1.endEdge1.p1 = cpl1.end1;
            //cpl1.endEdge1.p2 = cpl1.mid;
            //cpl1.endEdge2 = new Edge(1, 2);
            //cpl1.endEdge2.p1 = cpl1.mid;
            //cpl1.endEdge2.p2 = cpl1.end2;
            //cpl1.endEdge1.next = cpl1.endEdge2;
            //cpl1.endEdge2.prev = cpl1.endEdge1;

            mid = Point.getMidPoint(a1touch, a3touch);
            probeMid.changeVector(mid, probe.center).makeUnit().multiply(probe.radius);
            mid.assignTranslation(probe.center, probeMid);
            Arc cpl2 = new Arc(probe.center, probe.radius);
            cpl2.vrts.add(a3touch);
            cpl2.vrts.add(mid);
            cpl2.vrts.add(a1touch);


            tp = null;
            if (a1.tori.get(atom3) == null) {
                //System.out.println("corresponding rolling patch not found for" + atom1 + " " + atom3);
                //continue;
            } else {
                for (ToroidalPatch tor : a1.tori.get(atom3)) {
                    if (Point.distance(probe.center, tor.probe1) < 0.005 || Point.distance(probe.center, tor.probe2) < 0.005){
                        tp = tor;
                    }
                }
            }
            if (tp == null) {
                System.out.println("corresponding rolling patch not found for " + atom1 + " " + atom3);
            } else {
                tp.concavePatchArcs.add(cpl2);
                cpl2.torus = tp;
                if (tp.concavePatchArcs.size() == 2){
                    tp.concavePatchArcs.get(0).opposite = tp.concavePatchArcs.get(1);
                    tp.concavePatchArcs.get(1).opposite = tp.concavePatchArcs.get(0);
                }
            }
            v1.changeVector(a1touch, a3touch).makeUnit();//v1v2
            v2.changeVector(a2touch, a3touch).makeUnit();//v1mid
            v3.changeVector(probe.center, a3.sphere.center).makeUnit();//v1probe
            if (VectorUtil.determinantVec3(v1.getFloatData(), v2.getFloatData(), v3.getFloatData()) > 0.f) {
                ArcUtil.reverseArc(cpl2, true);
            }
            //cpl2.mid = mid;

            cpl2.setEndPoints(cpl2.vrts.get(0), cpl2.vrts.get(2), true);

            //cpl2.endEdge1 = new Edge(0, 1);
            //cpl2.endEdge1.p1 = cpl2.end1;
            //cpl2.endEdge1.p2 = cpl2.mid;
            //cpl2.endEdge2 = new Edge(1, 2);
            //cpl2.endEdge2.p1 = cpl2.mid;
            //cpl2.endEdge2.p2 = cpl2.end2;
            //cpl2.endEdge1.next = cpl2.endEdge2;
            //cpl2.endEdge2.prev = cpl2.endEdge1;

            mid = Point.getMidPoint(a2touch, a3touch);
            probeMid.changeVector(mid, probe.center).makeUnit().multiply(probe.radius);
            mid.assignTranslation(probe.center, probeMid);
            Arc cpl3 = new Arc(probe.center, probe.radius);
            cpl3.vrts.add(a2touch);
            cpl3.vrts.add(mid);
            cpl3.vrts.add(a3touch);

            tp = null;
            if (a2.tori.get(atom3) == null) {
            } else {
                for (ToroidalPatch tor : a2.tori.get(atom3)) {
                    if (Point.distance(probe.center, tor.probe1) < 0.005 || Point.distance(probe.center, tor.probe2) < 0.005){
                        tp = tor;
                    }
                }
            }
            if (tp == null) {
                System.out.println("corresponding rolling patch not found for " + atom2 + " " + atom3);
            } else {
                tp.concavePatchArcs.add(cpl3);
                cpl3.torus = tp;
                if (tp.concavePatchArcs.size() == 2){
                    tp.concavePatchArcs.get(0).opposite = tp.concavePatchArcs.get(1);
                    tp.concavePatchArcs.get(1).opposite = tp.concavePatchArcs.get(0);
                }
            }
            v1.changeVector(a3touch, a2touch).makeUnit();//v1v2
            v2.changeVector(a1touch, a2touch).makeUnit();//v1mid
            v3.changeVector(probe.center, a2.sphere.center).makeUnit();//v1probe
            if (VectorUtil.determinantVec3(v1.getFloatData(), v2.getFloatData(), v3.getFloatData()) > 0.f) {
                ArcUtil.reverseArc(cpl3, true);
            }
            //cpl3.mid = mid;

            cpl3.setEndPoints(cpl3.vrts.get(0), cpl3.vrts.get(2), true);

            //cpl3.endEdge1 = new Edge(0, 1);
            //cpl3.endEdge1.p1 = cpl3.end1;
            //cpl3.endEdge1.p2 = cpl3.mid;
            //cpl3.endEdge2 = new Edge(1, 2);
            //cpl3.endEdge2.p1 = cpl3.mid;
            //cpl3.endEdge2.p2 = cpl3.end2;
            //cpl3.endEdge1.next = cpl3.endEdge2;
            //cpl3.endEdge2.prev = cpl3.endEdge1;

            //List<Arc> q = new ArrayList<>();
            q.clear();
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
                if (Point.distance(pivot, l.end1) < 0.001) {
                    pivotLoop.next = l;
                    l.prev = pivotLoop;
                    //pivotLoop.endEdge2.next = l.endEdge1;
                    //l.endEdge1.prev = pivotLoop.endEdge2;
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
            } while (Point.distance(start, pivot) >= 0.0001);

            if (ghost) {
                cpl1.owner = cpatch;
                cpl2.owner = cpatch;
                cpl3.owner = cpatch;
                cpl1.valid = cpl2.valid = cpl3.valid = false;
                return;
            }
            cpl1.prev = pivotLoop;
            pivotLoop.next = cpl1;
            //pivotLoop.endEdge2.next = cpl1.endEdge1;
            //cpl1.endEdge1.prev = pivotLoop.endEdge2;
            Boundary b = new Boundary();
            b.arcs.add(cpl1);
            b.arcs.add(cpl1.next);
            b.arcs.add(cpl1.prev);
            cpl1.bOwner = cpl2.bOwner = cpl3.bOwner = b;


            cpatch.boundaries.add(b);
            b.patch = cpatch;

            cpl1.owner = cpatch;
            cpl2.owner = cpatch;
            cpl3.owner = cpatch;
            ArcUtil.refineArc(cpl1, Surface.maxEdgeLen, false, 0, false);
            ArcUtil.refineArc(cpl2, Surface.maxEdgeLen, false, 0, false);
            ArcUtil.refineArc(cpl3, Surface.maxEdgeLen, false, 0, false);
            cpl1.valid = cpl2.valid = cpl3.valid = true;
            ArcUtil.buildEdges(b, true);
            Surface.triangles.add(cpatch);
        } catch (Exception e){
            e.printStackTrace();
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
        Surface.rectangles.add(tp);
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

    //main method of this class, calls all of the methods used to parse, construct surface...
    public static void ses_start(String folder) {
        long _parseStartTime = System.currentTimeMillis();
        MeshGeneration.reset();
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
        Surface.convexPatches = SurfaceParser.parseAtomsBinary(Paths.get(folder).resolve("atoms.dat").toString());
        Surface.centerOfgravity.x /= SesConfig.atomCount;
        Surface.centerOfgravity.y /= SesConfig.atomCount;
        Surface.centerOfgravity.z /= SesConfig.atomCount;
        Surface.probeRadius.set(Double.doubleToLongBits(SesConfig.probeRadius));
        SurfaceParser.parseConvexAndToriPatchesBinary(Paths.get(folder).resolve("rectangles.dat").toString());
        try {
            SurfaceParser.parseTrianglesBinary(Paths.get(folder).resolve("triangles.dat").toString());

            constructProbeTree();
            Surface.probeTree.setIdenticalExcluded(true);

            PatchUtil.processSelfIntersectingTori();

            PatchUtil.processSelfIntersectingConcavePatches();


            PatchUtil.processIntersectingConcavePatches();
            //try {
            //    System.in.read();
            //} catch (Exception e){
            //    e.printStackTrace();
            //}
            ArcUtil.constructConvexBoundaries();
            ArcUtil.refineArcsOnSphericalPatches();

            //ArcUtil.refineArcsOnConvexPatches();

            ArcUtil.nestConvexPatchBoundaries();

            //ArcUtil.refineArcsOnConcavePatches();

            long _parseEndTime = System.currentTimeMillis();

            //try {
            //    System.in.read();
            //    System.out.println("After constructing");
            //} catch (Exception e){
            //    e.printStackTrace();
            //}
            if (SesConfig.useGUI) {
                MainWindow.mainWindow.sendPatchesLists(Surface.convexPatches, Surface.triangles);
            }
            //try {
            //    System.in.read();
            //    System.out.println("After gpu push");
            //} catch (Exception e){
            //    e.printStackTrace();
            //}
            MeshGeneration.startMeshing();
            while (!MeshGeneration.finished.get()){}
            //try {
            //    System.out.println("After mesh wait");
            //    System.in.read();
            //} catch (Exception e){
            //    e.printStackTrace();
            //}
            if (SesConfig.useGUI){
                MainWindow.mainWindow.pushTori();
                MainWindow.mainWindow.pushConvex();
                MainWindow.mainWindow.pushConcave();
            }
            if (SesConfig.objFile != null || SesConfig.stlFile != null){
                fillCommonVertices();
                while (!MeshGeneration.finished.get()){}
                if (SesConfig.objFile != null){
                    exportOBJ(SesConfig.objFile, (char)7);
                }
                if (SesConfig.stlFile != null){
                    exportSTLText(SesConfig.stlFile);
                }
            }
            if (SesConfig.verbose) {
                System.out.println("Convex patches count: " + Surface.convexPatches.size());
                System.out.println("Concave patches count: " + Surface.triangles.size());
                System.out.println("Toroidal patches count: " + Surface.rectangles.size());
                System.out.println("Average edge length: " + SesConfig.edgeLimit);
                System.out.println("Parsing and construction and trimming of boundaries took " + (_parseEndTime - _parseStartTime) + " milliseconds");
                int mbytes = 1024 * 1024;
                float maxHeapSize = (float) Runtime.getRuntime().maxMemory() / mbytes;
                float heapSize = (float) Runtime.getRuntime().totalMemory() / mbytes;
                float freeMemory = (float) Runtime.getRuntime().freeMemory() / mbytes;
                System.out.println("Heap size: " + heapSize + " / " + maxHeapSize + " MB");
                System.out.println("Heap usage: " + (heapSize - freeMemory) + " / " + heapSize + " MB");
            }
        } catch (Exception e){
            System.out.println(e.getMessage());
        }
    }

    public static void remesh(){
        MainWindow.mainWindow.requestFreeResources();
        while (!MainWindow.mainWindow.getResourcesFreed()){}
        MeshGeneration.reset();
        for (SphericalPatch sp : Surface.convexPatches){
            ArcUtil.resetArcs(sp);
            sp.meshed = false;
            //sp.faces.clear();
            //sp.faceCount = 0;
            //sp.vertices.clear();
        }
        //ArcUtil.refineArcsOnConvexPatches();
        for (SphericalPatch sp : Surface.triangles){
            ArcUtil.resetArcs(sp);
            sp.meshed = false;
            //sp.faces.clear();
            //sp.faceCount = 0;
        }
        ArcUtil.refineArcsOnSphericalPatches();
        //ArcUtil.refineArcsOnConcavePatches();
        MainWindow.mainWindow.sendPatchesLists(Surface.convexPatches, Surface.triangles);
        MeshGeneration.startMeshing();
        fillCommonVertices();
    }

    private static void fillCommonVertices(){
        Surface.commonVrts.clear();
        Surface.normals.clear();
        int idx = 1;
        for (SphericalPatch a : Surface.convexPatches){
            for (Boundary b : a.boundaries){
                for (Point p : b.vrts){
                    p.idx = idx++;
                    Surface.commonVrts.add(p);
                    Vector n = Point.subtractPoints(p, a.sphere.center).makeUnit();
                    Surface.normals.add(n);
                    //p.common = true;
                }
            }
        }
        for (SphericalPatch cp : Surface.triangles){
            for (Point p : cp.boundaries.get(0).vrts){
                p.idx = idx++;
                Surface.commonVrts.add(p);
                Vector n = Point.subtractPoints(cp.sphere.center, p).makeUnit();
                Surface.normals.add(n);
                //p.common = true;
            }
        }
    }

    private static int rollingCount = 0;

    //main export methods

    public static boolean exportOBJ(String filename, char mask){
        if (Surface.commonVrts.size() == 0){
            fillCommonVertices();
        }
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(filename))){
            int offset = 0;
            Vector _normal = new Vector(0, 0, 0);
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
                    if (a.faces == null){
                        continue;
                    }
                    List<Point> vrts = a.vertices;
                    //List<Integer> faces = a.faces;
                    int[] faces = a.faces;
                /*for (int i = 0; i < vrts.size(); ++i){

                }*/
                   // List<Vector> norms = new ArrayList<>();
                    int ownVerticesCount = 0;
                    for (Point p : vrts) {
                        if (p.idx > 0) {
                            continue;
                        }
                        //Vector n = Point.subtractPoints(p, a.sphere.center).makeUnit();
                        _normal.changeVector(p, a.sphere.center).makeUnit();
                        //norms.add(n);
                        bw.write("v " + p.toString());
                        bw.newLine();
                        bw.write("vn " + _normal.toString());
                        bw.newLine();
                        ownVerticesCount++;
                        p.ownIdx = ownIdx++;
                    }
                    if (!a.meshed){
                        continue;
                    }
                    for (int i = 0; i < faces.length; i += 3) {
                        Point p = vrts.get(faces[i]);
                        Point q = vrts.get(faces[i + 1]);
                        Point r = vrts.get(faces[i + 2]);
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
                    if (cp.faces == null){
                        continue;
                    }
                    List<Point> vrts = cp.vertices;
                    //List<Integer> faces = cp.faces;
                    int[] faces = cp.faces;
                    int ownVerticesCount = 0;
                    for (Point p : vrts) {
                        if (p.idx > 0) {
                            continue;
                        }
                        //Vector n = Point.subtractPoints(cp.sphere.center, p).makeUnit();
                        _normal.changeVector(cp.sphere.center, p);
                        bw.write("v " + p.toString());
                        bw.newLine();
                        bw.write("vn " + _normal.toString());
                        bw.newLine();
                        ownVerticesCount++;
                        p.ownIdx = ownIdx++;
                    }
                    if (!cp.meshed){
                        continue;
                    }
                    for (int i = 0; i < faces.length; i += 3){ //Face f : faces){
                        Point p = vrts.get(faces[i]);
                        Point q = vrts.get(faces[i + 1]);
                        Point r = vrts.get(faces[i + 2]);
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
                    if (tp.faces == null){
                        continue;
                    }
                    List<Point> vrts = tp.vertices;
                    List<Vector> normals = tp.normals;
                    //List<Face> faces = tp.faces;
                    //List<Integer> faces = tp.faces;
                    int[] faces = tp.faces;
                    int ownVerticesCount = 0;
                    for (int i = 0; i < vrts.size(); ++i) {
                        Point p = vrts.get(i);
                        if (p.idx > 0) {
                            continue;
                        }
                        _normal.changeVector(tp.probes[PatchUtil.getTorusProbeIdx(tp, i)], p).makeUnit();
                        bw.write("v " + p.toString());
                        bw.newLine();
                        //bw.write("vn " + normals.get(i).toString());
                        bw.write("vn " + _normal.toString());
                        bw.newLine();
                        ownVerticesCount++;
                        p.ownIdx = ownIdx++;
                    }
                    for (int i = 0; i < faces.length; i += 3){//Face f : faces) {
                        Point p = vrts.get(faces[i]);
                        Point q = vrts.get(faces[i + 1]);
                        Point r = vrts.get(faces[i + 2]);
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
        } catch (IOException e){
            e.printStackTrace();
            return false;
        }
        return true;
    }

//    public static boolean exportSTL(String file){
//        try (DataOutputStream ds = new DataOutputStream(new FileOutputStream(file))){
//            /*for (int i = 0; i < 20; ++i){
//                ds.writeInt(i);
//            }*/
//            for (int i = 0; i < 80; ++i){
//                ds.writeByte(0);
//            }
//            ds.writeInt(Surface.numoftriangles);
//            List<Point> vrts;
//            List<Vector> normals;
//            List<Face> faces;
//            for (SphericalPatch a : Surface.convexPatches){
//                vrts = a.vertices;
//                faces = a.faces;
//                for (Face f : faces) {
//                    ds.writeFloat(0.f);
//                    ds.writeFloat(0.f);
//                    ds.writeFloat(0.f);
//
//                    Point p = vrts.get(f.c);
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    p = vrts.get(f.b);
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    p = vrts.get(f.a);
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    ds.writeShort(0);
//                }
//            }
//            for (SphericalPatch cp : Surface.triangles){
//                vrts = cp.vertices;
//                faces = cp.faces;
//                for (Face f : faces) {
//                    ds.writeFloat(0.f);
//                    ds.writeFloat(0.f);
//                    ds.writeFloat(0.f);
//
//                    Point p = vrts.get(f.a);
//                    ds.writeFloat((float)p.x + (float)0.f);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    p = vrts.get(f.b);
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    p = vrts.get(f.c);
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    ds.writeShort(0);
//                }
//            }
//            for (ToroidalPatch rp : Surface.rectangles){
//                vrts = rp.vertices;
//                List<Integer> _faces = rp.faces;
//                for (int i = 0; i < _faces.size(); i += 3){//Face f : faces) {
//                    ds.writeFloat(0.f);
//                    ds.writeFloat(0.f);
//                    ds.writeFloat(0.f);
//
//                    Point p = vrts.get(_faces.get(i + 2));
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    p = vrts.get(_faces.get(i + 1));
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    p = vrts.get(_faces.get(i));
//                    ds.writeFloat((float)p.x + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.y + (float) Surface.stlXOffset);
//                    ds.writeFloat((float)p.z + (float) Surface.stlXOffset);
//                    ds.writeShort(0);
//                }
//            }
//            ds.flush();
//            ds.close();
//        } catch (IOException e){
//            e.printStackTrace();
//            return false;
//        }
//        return true;
//    }

    public static boolean exportSTLText(String file){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(file))){
            List<Point> vrts;
            int[] faces;
            bw.write("solid ");
            bw.newLine();
            for (SphericalPatch a : Surface.convexPatches){
                if (a.faces == null){
                    continue;
                }
                vrts = a.vertices;
                faces = a.faces;
                for (int i = 0; i < a.faces.length; i += 3){ //Face f : faces){
                    bw.write("facet normal 0.0 0.0 0.0");
                    bw.newLine();
                    bw.write("outer loop");
                    bw.newLine();
                    bw.write("vertex " + vrts.get(faces[i]).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(faces[i + 1]).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(faces[i + 2]).toString());
                    bw.newLine();
                    bw.write("endloop");
                    bw.newLine();
                    bw.write("endfacet");
                    bw.newLine();
                }
            }
            for (SphericalPatch cp : Surface.triangles){
                if (cp.faces == null){
                    continue;
                }
                vrts = cp.vertices;
                faces = cp.faces;
                for (int i = 0; i < cp.faces.length; i += 3){ //Face f : faces){
                    bw.write("facet normal 0.0 0.0 0.0");
                    bw.newLine();
                    bw.write("outer loop");
                    bw.newLine();
                    bw.write("vertex " + vrts.get(faces[i + 2]).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(faces[i + 1]).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(faces[i]).toString());
                    bw.newLine();
                    bw.write("endloop");
                    bw.newLine();
                    bw.write("endfacet");
                    bw.newLine();
                }
            }
            for (ToroidalPatch rp : Surface.rectangles){
                if (rp.faces == null){
                    continue;
                }
                vrts = rp.vertices;
                int[] _faces = rp.faces;
                for (int i = 0; i < _faces.length; i += 3){//Face f : faces){
                    bw.write("facet normal 0.0 0.0 0.0");
                    bw.newLine();
                    bw.write("outer loop");
                    bw.newLine();
                    bw.write("vertex " + vrts.get(_faces[i]).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(_faces[i + 1]).toString());
                    bw.newLine();
                    bw.write("vertex " + vrts.get(_faces[i + 2]).toString());
                    bw.newLine();
                    bw.write("endloop");
                    bw.newLine();
                    bw.write("endfacet");
                    bw.newLine();
                }
            }
            bw.write("endsolid");
            bw.flush();
        } catch (IOException e){
            e.printStackTrace();
            return false;
        }
        return true;
    }


    //these have just a debug purpose
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

    public static void exportPatch(SphericalPatch sp){
        try (BufferedWriter bw = new BufferedWriter(new FileWriter("/home/radoslav/objs/patch" + sp.id + (Math.random() * 10) + ".obj"))) {
            String line = "";
            for (Point v : sp.vertices){
                line = "v " + v.toString();
                bw.write(line);
                bw.newLine();
                line = "vn " + Point.subtractPoints(v, sp.sphere.center).makeUnit().toString();
                bw.write(line);
                bw.newLine();
            }
            for (int i = 0; i < sp.faces.length; i += 3){//Face f : sp.faces){
                int i1 = sp.faces[i] + 1;
                int i2 = sp.faces[i + 1] + 1;
                int i3 = sp.faces[i + 2 ]+ 1;
                line = "f " + i1 + "//" + i1 + " " + i2 + "//" + i2 + " " + i3 + "//" + i3;
                bw.write(line);
                bw.newLine();
            }
            bw.flush();
        } catch (IOException e){
            e.printStackTrace();
        }
    }

//    public static void exportToroidalPatch(ToroidalPatch tp){
//        try (BufferedWriter bw = new BufferedWriter(new FileWriter("tori_" + tp.id + "-" + LocalDateTime.now().toString() + ".obj"))){
//            for (Point v : tp.vertices){
//                bw.write("v " + v.toString());
//                bw.newLine();
//            }
//            for (Vector n : tp.normals){
//                bw.write("vn " + n.toString());
//                bw.newLine();
//            }
//            for (int i = 0; i < tp.faces.size(); i += 3){//Face f : tp.faces){
//                bw.write("f " + (tp.faces.get(i) + 1) + "//" + (tp.faces.get(i) + 1) + " " + (tp.faces.get(i + 1) + 1) + "//" + (tp.faces.get(i + 1) + 1) + " " + (tp.faces.get(i + 2) + 1) + "//" + (tp.faces.get(i + 2) + 1));
//                bw.newLine();
//            }
//            bw.flush();
//        } catch (IOException e){
//            e.printStackTrace();
//        }
//    }

//    public static void exportOldFaces(SphericalPatch sp){
//        try (BufferedWriter bw = new BufferedWriter(new FileWriter("/home/radoslav/objs/patch" + sp.id + (Math.random() * 10) + ".obj"))) {
//            String line = "";
//            for (Point v : sp.vertices){
//                line = "v " + v.toString();
//                bw.write(line);
//                bw.newLine();
//                line = "vn " + Point.subtractPoints(v, sp.sphere.center).makeUnit().toString();
//                bw.write(line);
//                bw.newLine();
//            }
//            for (Face f : sp.dbFaces){
//                int i1 = f.a + 1;
//                int i2 = f.b + 1;
//                int i3 = f.c + 1;
//                line = "f " + i1 + "//" + i1 + " " + i2 + "//" + i2 + " " + i3 + "//" + i3;
//                bw.write(line);
//                bw.newLine();
//            }
//            bw.flush();
//        } catch (IOException e){
//            e.printStackTrace();
//        }
//    }

//    public static void exportCP_(SphericalPatch sp){
//        try (BufferedWriter bw = new BufferedWriter(new FileWriter("/home/radoslav/objs/newB_" + sp.id + ".obj"))){
//            int off = 0;
//            for (Boundary b : sp.boundaries){
//                Boundary b_ = new Boundary();
//                for (Arc a : b.arcs){
//                    b_.arcs.add(a.refined);
//                }
//                ArcUtil.buildEdges(b_, true);
//                for (Point v : b_.vrts){
//                    bw.write("v " + v.toString());
//                    bw.newLine();
//                }
//                for (int i = 1; i <= b_.vrts.size(); ++i){
//                    if (i == b_.vrts.size()){
//                        bw.write("l " + (i + off) + " " + (1 + off));
//                    } else {
//                        bw.write("l " + (i + off) + " " + (i + 1 + off));
//                    }
//                    bw.newLine();
//                }
//                off += b_.vrts.size();
//            }
//        } catch (IOException e){
//            e.printStackTrace();
//        }
//    }
}
