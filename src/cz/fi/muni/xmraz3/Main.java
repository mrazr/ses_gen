package cz.fi.muni.xmraz3;


import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import smile.neighbor.KDTree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

public class Main {
    public static  double maxEdgeLen = 0.3;
    public static IntegerProperty atomsProcessed = new SimpleIntegerProperty(0);
    public static List<SphericalPatch> convexPatches;
    public static AtomicLong probeRadius = new AtomicLong(Double.doubleToLongBits(1.4));
    public static List<SphericalPatch> triangles = new ArrayList<>();
    public static List<ToroidalPatch> rectangles = new ArrayList<>();
    public static List<ToroidalPatch> smallRectangles = new ArrayList<>();
    public static List<ToroidalPatch> selfIntersectingRects = new ArrayList<>();
    public static List<Arc> intersectingArcs = new ArrayList<>();
    public static List<Point> commonVrts = new ArrayList<>();
    public static List<Vector> normals = new ArrayList<>();
    public static float stlXOffset = 10000.f;
    public static int stlYOffset = 1000;
    public static int stlZOffset = 1000;
    public static int numoftriangles = 0;
    public static KDTree<SphericalPatch> probeTree;
    public static Point centerOfgravity = new Point(0., 0., 0.);
    public static double scaleFactor = 1.;

    private static void processSelfIntersectingTori(){
        for (ToroidalPatch tp : selfIntersectingRects){
            PatchUtil.torProcessSelfIntersection(tp);
        }
    }

    public static void ses_start(String folder) {
        triangles.clear();
        rectangles.clear();
        atomsProcessed.set(0);
        selfIntersectingRects.clear();
        intersectingArcs.clear();
        numoftriangles = 0;
        SphericalPatch.nextConcaveID = SphericalPatch.nextConvexID = 0;
        ToroidalPatch.nextID = 0;
        //String folder = "./crn26-139/";
        /*String jsonAtom = SurfaceParser.loadFile(folder + "atoms.json");
        String jrectangles = SurfaceParser.loadFile(folder + "rectangles.json");
        String jtriangles = SurfaceParser.loadFile(folder + "triangles.json");*/
        //convexPatches = SurfaceParser.parseForAtoms(jsonAtom);
        convexPatches = SurfaceParser.parseAtoms(folder + "atoms.dat");
        centerOfgravity.x /= SesConfig.atomCount;
        centerOfgravity.y /= SesConfig.atomCount;
        centerOfgravity.z /= SesConfig.atomCount;
        System.out.println(centerOfgravity.toString());
        //helperthing = new ArrayList<List<PointRadius>>(convexPatches.size());
        /*for (int i = 0; i < convexPatches.size(); ++i){
            helperthing.add(new ArrayList<>());
        }*/
        //String jsonConvex = SurfaceParser.loadFile(folder + "convexPatches.json");
        //SurfaceParser.parseConvexPatchB(jsonConvex);
        probeRadius.set(Double.doubleToLongBits(SesConfig.probeRadius));
        System.out.println("probe: " + Double.longBitsToDouble(probeRadius.get()));
        System.out.println("probe2: " + SesConfig.probeRadius);
        //SurfaceParser.jsonParseConvexAndToriPatches(jrectangles, convexPatches, Double.longBitsToDouble(probeRadius.get()));
        SurfaceParser.parseConvexAndToriPatches(folder + "rectangles.dat");
        System.out.println(selfIntersectingRects.size() + " selfintersecting triangles");
        /*for (RollingPatch rp : selfIntersectingRects){
            for (RollingPatch r : smallRectangles){
                if (rp == r){
                    System.out.println("small == self");
                }
            }
        }*/
        //SurfaceParser.jsonParseTriangles(jtriangles);

        try {
            SurfaceParser.parseTriangles(folder + "triangles.dat");
            constructProbeTree();
            probeTree.setIdenticalExcluded(true);

            System.out.println("Number of triangles: " + triangles.size());
            //BufferedWriter bw = new BufferedWriter(new FileWriter(fileName, true));
            int offset = 0;
            System.out.println("Num of atoms: " + convexPatches.size());
            int vrtsOffset = 0;
            int ooo = 0;
            //ConcavePatchUtil.removeSmallRectangles();

            for (int i = 0; i < convexPatches.size(); ++i) {
                int offset2 = 0;
                SphericalPatch a = convexPatches.get(i);

                //a.linkLoops();
                ArcUtil.linkArcs(a);

                vrtsOffset = 0;
                for (Boundary b : a.boundaries){
                    for (Boundary c : a.boundaries){
                        if (c == b){
                            continue;
                        }
                        if (ArcUtil.checkIfNested(b, c)){
                            b.nestedBoundaries.add(c);
                            //System.out.println("atom " + a.id + " probably has nested boundaries");
                            //System.out.println(" ");
                        }
                    }
                    int j = 0;
                    /*for (Arc l : b.arcs) {
                        //l.refineLoop(3);
                        j++;
                        l.refineLoop(maxEdgeLen, true);
                        //SurfaceParser.exportOBJ(l, bw, offset);
                        //SurfaceParser.exportOBJ(l, bw2, offset2);
                        //offset += l.vrts.size();
                        //offset2 += l.vrts.size();
                        if (b.atom.id == 84){
                            SurfaceParser.exportOBJ2(l, "loop84" + ooo + ".obj", false, 0);
                            ooo++;
                        }*/
                    //b.vrts = new ArrayList<>();
                    //b.lines = new ArrayList<>();
                    Point start = b.arcs.get(0).end1;
                    Edge e = b.arcs.get(0).endEdge1;
                    do {
                        b.vrts.add(e.p1);
                        e = e.next;
                    } while (e.p1 != start);
                    for (Arc el : b.arcs){
                        //b.vrts.addAll(el.vrts);
                        /*for (int k = 0; k < el.vrts.size() - 1; ++k){
                            b.vrts.add(el.vrts.get(k));
                        }
                        for (Edge e : el.lines){
                            e.v1 += vrtsOffset;
                            e.v2 += vrtsOffset;
                            b.lines.add(e);
                        }
                        vrtsOffset += el.vrts.size();//el.getVertices().size() - 1;*/
                    }
                    //b.buildEdges(false);
                    ArcUtil.buildEdges(b, false);
                    //SurfaceParser.exportOBJ2(b, "b" + b.atom.id + "" + b.vrts.size() + ".obj", false, 0);
                }


                /*for (Boundary b : a.convexPatchBoundaries) {
                    int nest = 0;
                    for (Boundary c : b.insideBoundaries){
                        String bFile = folder + "/out/katom_" + i + "_nestedB" + nest + ".obj";
                        BufferedWriter bww = new BufferedWriter(new FileWriter(bFile, true));
                        int boffset = 0;
                        for (Arc l : c.arcs){
                            SurfaceParser.exportOBJ(l, bww, boffset);
                            boffset += l.vrts.size();
                        }
                        for (Arc l : b.arcs){
                            SurfaceParser.exportOBJ(l, bww, boffset);
                            boffset += l.vrts.size();
                        }
                        nest++;
                        bww.close();
                    }
                }*/

                //System.out.println("i: " + i);
            /*if (a.convexPatchBoundaries.size() > 0) {
                Boundary b = a.convexPatchBoundaries.get(0);
                offset = 0;
                for (Arc l : b.arcs) {
                    l.refineLoop(2);
                    SurfaceParser.exportOBJ2(l, "atom" + i + ".obj", true, offset);
                    offset += l.vrts.size();
                }
            }*/
                //bw2.close();
                atomsProcessed.set(i + 1);
            }



            for (SphericalPatch cp : triangles){
                Boundary b = cp.boundaries.get(0);
                //vrtsOffset = 0;
                for (Arc k : b.arcs){
                    Arc c = k;
                    //c.owner = cp;
                    //TO DO - the two lines below can be executed in the parsing method!
                    //c.refineLoop(maxEdgeLen, 0.0, false, 0, false);
                    //c.buildEdges();
                    //ArcUtil.refineArc(c, maxEdgeLen, false,0, false);
                    ArcUtil.buildEdges(c);
                    for (int l = 0; l < k.vrts.size() - 1; ++l){
                        b.vrts.add(k.vrts.get(l));
                    }
                    ooo++;
                    for (Edge e : c.lines){
                        if (e == null){
                            System.out.println( " ");
                        }
                        e.v1 += vrtsOffset;
                        e.v2 += vrtsOffset;
                        b.lines.add(e);
                    }
                    vrtsOffset += c.vrts.size() - 1;
                }
                //b.buildEdges(false);
                ArcUtil.buildEdges(b, false);
            }
            processSelfIntersectingTori();
            processSelfIntersectingConcavePatches();
            PatchUtil.processIntersectingConcavePatches();
            /*for (ConcavePatch cp : triangles){
                if (!cp.valid){
                    continue;
                }
                //ConcavePatchUtil.analyze34(cp);
                ConcavePatchUtil.analyzeForIntersections(cp);
            }*/

            /*for (ConcavePatch cp : triangles){
                List<Neighbor<double[], ConcavePatch>> neighs = new ArrayList<>();
                probeTree.range(cp.probe.getData(), SesConfig.probeRadius * 2, neighs);
                for (Neighbor<double[], ConcavePatch> cp2 : neighs){
                    ConcavePatchUtil.analyze(cp, cp2.value);
                }
            }*/
            //System.out.println("started meshing rects");
            /*for (RollingPatch rp : rectangles){
            //    rp.mesh();
            }*/
            //System.out.println("meshed " + RollingPatch.count + " selfintersecting rects");
            int i = 0;
            for (SphericalPatch cp : triangles){
                cp.id = i;
                i++;
                Boundary b = cp.boundaries.get(0);
                b.vrts.clear();
                b.lines.clear();
                vrtsOffset = 0;

                /*if (b.arcs.size() > 3){
                    System.out.println("loop id: " + (i - 1));
                }*/
                /*if (cp.merged){
                    System.out.println("merged: " + i);
                }*/
                for (Arc k : b.arcs){
                    if (k == null){
                        System.out.println(" ");
                    }
                    Arc c = k;
                    //b.vrts.addAll(k.vrts);
                    for (int l = 0; l < k.vrts.size() - 1; ++l){
                        b.vrts.add(k.vrts.get(l));
                    }
                    ooo++;
                    for (Edge e : c.lines){
                        if (e == null){
                            System.out.println( " ");
                        }
                        e.v1 += vrtsOffset;
                        e.v2 += vrtsOffset;
                        //b.lines.add(e);
                    }
                    vrtsOffset += c.vrts.size() - 1;
                }
                //b.buildEdges(false);
                ArcUtil.buildEdges(b, false);
            }
            /*for (ConcavePatch cp : triangles){
                ConcavePatchUtil.analyze2(cp);
            }*/

            System.out.println("done meshing rects");
            //atomsProcessed.set(convexPatches.size());
            //bw.close();
            fillCommonVertices();
            System.out.println("Hotovo");
        } catch (Exception e){
            System.out.println(e.getMessage());
        }
    }

    private static void constructProbeTree(){
        double[][] keys = new double[triangles.size()][3];
        SphericalPatch values[] = new SphericalPatch[triangles.size()];
        for (int i = 0; i < keys.length; ++i){
            Point p = triangles.get(i).sphere.center;
            keys[i] = p.getData();
            values[i] = triangles.get(i);
        }
        probeTree = new KDTree<>(keys, values);
    }

    private static void fillCommonVertices(){
        commonVrts.clear();
        int idx = 1;
        for (SphericalPatch a : convexPatches){
            for (Boundary b : a.boundaries){
                for (Point p : b.vrts){
                    p.idx = idx++;
                    commonVrts.add(p);
                    Vector n = Point.subtractPoints(p, a.sphere.center).makeUnit();
                    normals.add(n);
                    p.common = true;
                }
            }
        }
        for (SphericalPatch cp : triangles){
            for (Point p : cp.boundaries.get(0).vrts){
                p.idx = idx++;
                commonVrts.add(p);
                Vector n = Point.subtractPoints(cp.sphere.center, p).makeUnit();
                normals.add(n);
                p.common = true;
            }
        }
    }

    private static void processSelfIntersectingConcavePatches() {
        try {
            for (Arc cpl : intersectingArcs) {
                //processSelfIntersectingConcavePatch(cpl);
                //PatchUtil.processIntersectingArcsOnPatch(cpl);
                PatchUtil.trimSelfIntersectingPatch(cpl);
            }
        } catch(Exception e){
            e.printStackTrace();
        }
    }

}
