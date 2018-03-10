package cz.fi.muni.xmraz3.mesh;

import cz.fi.muni.xmraz3.SesConfig;
import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.utils.ArcUtil;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MeshRefinement {
    public static MeshRefinement refinement = new MeshRefinement();
    private static int concaveRefined = 0;
    private static int convexRefined = 0;
    private static int convexMeshed = 0;
    private static int concaveMeshed = 0;
    private static Queue<SphericalPatch> phase1Queue = new ConcurrentLinkedQueue<>();
    private static Queue<SphericalPatch> phase2Queue = new ConcurrentLinkedQueue<>();
    private boolean running = false;

    public void start(){
        Runnable r = new Runnable() {
            @Override
            public void run() {
                phase1Refine();
            }
        };
        (new Thread(r)).start();
        running = true;
    }

    public boolean isRunning(){
        return running;
    }

    private static void phase1Refine(){
        while (concaveRefined < Surface.triangles.size() || convexRefined < Surface.convexPatches.size()){
            while (phase1Queue.isEmpty());
            SphericalPatch sp = phase1Queue.poll();
            if (sp.convexPatch){
                for (Boundary b : sp.boundaries){
                    for (Arc a1 : b.arcs){
                        if (a1.refined != null){
                            continue;
                        }
                        Arc op = a1.opposite;
                        a1.refined = ArcUtil.dbgCloneArc(a1);
                        op.refined = ArcUtil.dbgCloneArc(op);
                        ArcUtil.refineOppositeArcs(a1.refined, op.refined, SesConfig.edgeLimit);
                        int a = 42;
                    }
                }
                convexRefined++;
            } else {
                for (Boundary b : sp.boundaries){
                    for (Arc a : b.arcs){
                        a.refined = ArcUtil.dbgCloneArc(a);
                        ArcUtil.refineArc(a.refined, SesConfig.edgeLimit, false, 0 ,false);
                    }
                }
                concaveRefined++;
            }
            phase2Queue.add(sp);
        }
    }

    public void enqueue(SphericalPatch sp){
        if (!sp.valid){
            if (sp.convexPatch){
                convexRefined++;
                convexMeshed++;
            } else {
                concaveRefined++;
                concaveMeshed++;
            }
        } else {
            phase1Queue.add(sp);
        }
    }

    /*public static void phase2Refine(){
        while (convexMeshed < Surface.convexPatches.size() || concaveMeshed < Surface.triangles.size()){

        }
    }*/
    private MeshRefinement(){}
}
