package cz.fi.muni.xmraz3.gui;


import com.jogamp.newt.MonitorDevice;
import com.jogamp.newt.Screen;
import com.jogamp.newt.event.*;
import com.jogamp.newt.opengl.GLWindow;
import com.jogamp.opengl.*;
import com.jogamp.opengl.math.Matrix4;
import com.jogamp.opengl.math.Quaternion;
import com.jogamp.opengl.math.VectorUtil;
import com.jogamp.opengl.util.Animator;
import com.jogamp.opengl.util.GLBuffers;
import com.jogamp.opengl.util.PMVMatrix;
import com.jogamp.opengl.util.texture.Texture;
import cz.fi.muni.xmraz3.*;
import cz.fi.muni.xmraz3.gui.controllers.MainPanelController;
import cz.fi.muni.xmraz3.math.Point;
import cz.fi.muni.xmraz3.math.Vector;
import cz.fi.muni.xmraz3.mesh.*;
import cz.fi.muni.xmraz3.utils.ArcUtil;
import cz.fi.muni.xmraz3.utils.GLUtil;
import cz.fi.muni.xmraz3.utils.PatchUtil;
import graphicslib3D.*;
import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.stage.Stage;
import smile.neighbor.Neighbor;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class MainWindow implements GLEventListener, KeyListener, MouseListener{
    public static MainWindow mainWindow = null;
    public GLWindow window;
    private Animator animator;
    private int mainProgram;
    private int rectProgram;
    public Stage controlPanel;
    //public boolean isPinned = false;
    public BooleanProperty isPinned = new SimpleBooleanProperty(false);
    GL4 gl;

    List<Point> vrts;
    List<Edge> lines;
    boolean isNew = false;

    private float[] data = {
            0.f, 0.f, -1.f,
            0.25f, 0.f, -1.f,
            0.f, 0.25f, -1.f,
            0.25f, 0.25f, -1.f
    };
    private int[] indices = {
            /*0, 2, 1,
            1, 2, 3*/
            0, 2,
            2, 1,
            1, 0,

            1, 2,
            2, 3,
            3, 1
    };

    private int[] vao = new int[1];
    private int[] vbo = new int[1];
    private int[] ebo = new int[1];
    private int numOfVrts = 0;
    private int numOfIndices = 0;
    boolean buffersInitialized = false;

    boolean captureMouse = false;

    //view params
    private float[] cameraPos = new float[] {0.f, 0.f, 0.f};
    private long trianglesCount = 0;
    float horizontalAngle = 3.14f;
    float verticalAngle = 0.0f;
    //private float[] direction = {(float)(Math.cos(verticalAngle) * Math.sin(horizontalAngle)), (float)Math.sin(verticalAngle), (float)(Math.cos(verticalAngle) * Math.cos(horizontalAngle))};
    private float[] direction = {0.f, 0.f, -1.0f};
    //private float[] right = {(float)Math.sin(horizontalAngle - 3.14f/2.0f), 0f, (float)Math.cos(horizontalAngle - 3.14f/2.0f)};
    private float[] up = {0.0f, 1.0f, 0.0f};
    private float[] right = new float[3];
    float initialFoV = 45.f;
    float speed = .03f;
    float mouseSpeed = 0.02f;
    private boolean dontConsider = false;
    float deltaTime = 0;
    float mouseAngleX = 0.f;
    float mouseAngleY = 0.f;
    float angle = 0.0f;
    private int pvLoc = -1;
    private int modelLoc = -1;
    private float modelX = 0.f;
    private float modelY = 0.f;
    private float modelZ = -2.f;
    private long lastTick;
    private int atomIdx = 0;
    Matrix3D pMat;
    Point3D cameraTarget;
    Point3D lightPos;

    private List<SphericalPatch> convexPatchList;
    private boolean newAtoms = false;
    //private int selectedAtom = 0;
    private int hoverAtom = -1;
    public IntegerProperty selectedAtom = new SimpleIntegerProperty(1);
    public IntegerProperty selectedConcaveP = new SimpleIntegerProperty(0);
    public IntegerProperty selectedToriP = new SimpleIntegerProperty(0);
    private String strSelectedAtom = "";
    private boolean selectedExclusiveRender = false;
    private AdvancingFrontMethod afm;
    AtomicBoolean update2 = new AtomicBoolean(false);
    private boolean update = false;
    private List<Edge> updaLines = new ArrayList<>();
    private List<Point> updaVrts = new ArrayList<>();
    private List<Face> updaFaces = new ArrayList<>();
    private boolean drawFaces = true;
    private boolean drawLinesOnTop = true;
    private boolean step = true;
    private boolean[] atomsMeshed;
    private boolean[] concavePatchesMeshed;

    private List<SphericalPatch> concavePatchList;
    private boolean newCPs = false;
    private boolean renderCPs = false;
    private AdvancingFrontMethod cpAfm;
    private List<Point> cpVrts = new ArrayList<>();
    private List<Face> cpFaces = new ArrayList<>();
    private AtomicBoolean cpUpdate = new AtomicBoolean(false);


    private float[] toriPatchCol = {33 / 255.f, 161 / 255.f, 235 / 255.f};
    //private float[] concavePatchSelCol = {234 / 255.f, 165 / 255.f, 33 / 255.f};
    private float[] concavePatchCol = {118 / 255.f, 220 / 255.f, 33 / 255.f};
    //private float[] convexPatchSelCol = {234 / 255.f, 165 / 255.f, 33 / 255.f};
    private float[] convexPatchCol = {1.0f, 89 / 255.f, 100 / 255.f};
    private float[] selectedPatchCol = {1.f, 231 / 255.f, 76 / 255.f};
    private float[] hoveredPatchCol = {.8f, .8f, .8f};
    private float[] pointColor = {137 / 255.f, 66 / 255.f, 244 / 255.f};
    private float[] clearColor = {1.f, 1.f, 1.f, 1.f};
    private boolean onlyCircular = false;

    private boolean mouseDown = true;
    private boolean zooming = false;
    private float zoomSpeed = 0.1f;
    private boolean cullFaces = true;
    private boolean viewPanning = false;
    private boolean raySelection = false;
    private Vector arcStart;
    private boolean mouseSelect = false;
    private int mouseSelectVbo[] = new int[1];
    private int mouseSelectVao[] = new int[1];
    private Point mouseLocation;

    Quaternion camDir;
    Quaternion camTar;
    private boolean slerping = false;
    private float slerpParam = 0.0f;

    private int[] fbo = new int[1];
    private int[] rbCol = new int[1];
    private int[] rbDep = new int[1];
    private int selProgram = -1;
    private boolean moved = false;
    private boolean renderBuffersInit = false;
    private int uniVertexOffsets = -1;
    private int uniGlobalOffset = -1;
    private int uniStart = -1;
    private int uniEnd = -1;
    private int convexVerticesCount = 0;
    private int concaveVerticesCount = 0;
    private int toriVerticesCount = 0;
    private Texture vertexOffsets;
    private boolean selectInitialized = false;
    private int[] tbo = new int[1];
    private int[] tboTex = new int[1];

    private long fpsLastTick = 0;
    private List<Integer> convexPatchesSelect = new ArrayList<>();
    private List<Integer> toriPatchesSelect = new ArrayList<>();
    private List<Integer> concavePatchesSelect = new ArrayList<>();

    private final static int CONVEX = 0;
    private final static int CONCAVE = 1;
    private final static int TORUS = 2;

    private int convexPatchesFaceCount = 0;
    private int concavePatchesFaceCount = 0;
    private int toriPatchesFaceCount = 0;

    private int[] meshVao = new int[3];
    private int[] meshVbo = new int[3];
    private int[] meshEbo = new int[2];

    private AtomicInteger convexMeshThreadsCounter = new AtomicInteger(0);
    private AtomicInteger concaveMeshThreadsCounter = new AtomicInteger(0);
    private AtomicInteger toriMeshThreadsCounter = new AtomicInteger(0);
    private final int threadCount = 4;

    private boolean convexMeshInitialized = false;
    private boolean concaveMeshInitialized = false;
    private boolean toriMeshInitialized = false;

    private int concaveFaceCountShow = 0;

    private int[] lineVao = new int[2];
    private int[] lineVbo = new int[2];
    private int[] lineEbo = new int[2];

    private int convexPatchesEdgeCount = 0;
    private int concavePatchesEdgeCount = 0;

    private AtomicBoolean convexPushData2GPU = new AtomicBoolean(false);
    private AtomicBoolean concavePushData2GPU = new AtomicBoolean(false);
    private AtomicBoolean toriPushData2GPU = new AtomicBoolean(false);

    private int[] probeVao = new int[1];
    private int[] probeVbo = new int[1];
    private int probeFaceCount = 0;
    private PMVMatrix probeScaleT = new PMVMatrix();
    private boolean renderProbe = false;
    private float probeAlpha = 0.4f;

    private List<Integer> linkNeighbors = new ArrayList<>();
    private Matrix4 selectedMeshScaleMat = new Matrix4();
    private float scaleFactor = 1.0f;
    private Point lastCameraTarget = null;
    private boolean drawModeUpdate;
    private PMVMatrix modelMAT = new PMVMatrix();
    private Matrix4 normalMatrix = new Matrix4();
    private double angleX = 0.0;
    private double angleY = 0.0;
    private int lastX;
    private int lastY;
    private boolean rotating = true;

    //pure opengl data
    private List<Integer> vaos;
    private List<Integer> vbos;
    private List<Integer> ebos;
    private int uniMeshColorLoc = -1;
    private int uniViewMatLoc = -1;
    private int uniProjMatLoc = -1;
    private int uniModelMatLoc = -1;
    private int uniAlphaLoc = -1;
    private int uniNormalColorLoc = -1;
    private int uniSelectedColorLoc = -1;
    private int uniSelectedMeshStartLoc = -1;
    private int uniSelectedMeshEndLoc = -1;
    private int uniSelectedMeshCountLoc = -1;
    private int uniAmbientStrengthLoc = -1;
    private int uniMvInverseLoc = -1;
    private IntBuffer zeroes = GLBuffers.newDirectIntBuffer(new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});

    private boolean render = false;
    private AtomicBoolean stopRendering = new AtomicBoolean(true);
    private AtomicBoolean stoppedRendering = new AtomicBoolean(true);
    private boolean addToSelection;
    private boolean removeSelection;
    private boolean renderLines = true;
    private AtomicBoolean resourcesFreed = new AtomicBoolean(true);
    private int convexFaceCountShow;


    public void sendPatchesLists(List<SphericalPatch> convex, List<SphericalPatch> concave){
        GLRunnable task = new GLRunnable() {
            @Override
            public boolean run(GLAutoDrawable glAutoDrawable) {
                freeGLResources();

                ///while (!stoppedRendering.get());
                sendConvexPatchList(convex);
                sendConcavePatchList(concave);
                //sendConvexPatches2GPU();
                //sendToriPatches2GPU();
                //sendConcavePathes2GPU();
                pushConcaveEdgesToGPU();
                pushConvexEdgesToGPU();
                //render = true;
                //animator.start();
                stopRendering.set(false);
                stoppedRendering.set(false);
                resourcesFreed.set(false);
                return true;
            }
        };
        window.invoke(false, task);
    }

    private void pushConvexEdgesToGPU(){
        int totalVerticesCount = 0;
        int totalIndicesCount = 0;
        int indexOffset = 0;
        int vertexOffset = 0;
        List<Point> vertices = new ArrayList<>();
        List<Integer> indices = new ArrayList<>();
        for (SphericalPatch a : convexPatchList){
            a.lineOffset = indexOffset;
            for (Boundary b : a.boundaries){
                Boundary b_ = new Boundary();
                b_.patch = a;
                for (Arc _a : b.arcs){
                    b_.arcs.add(_a.refined);
                }
                ArcUtil.buildEdges(b_, true);
                a.lineCount += b_.lines.size();
                for (Point p : b_.vrts){
                    vertices.add(p);
                }
                for (Edge e : b_.lines){
                    indices.add(e.v1 + vertexOffset);
                    indices.add(e.v2 + vertexOffset);
                }
                vertexOffset += b_.vrts.size();
                //totalVerticesCount += b.vrts.size();
            }
            indexOffset += a.lineCount;
        }
        totalVerticesCount = vertices.size();
        totalIndicesCount = indices.size();
        convexPatchesEdgeCount = totalIndicesCount / 2;
        FloatBuffer vBuff = GLBuffers.newDirectFloatBuffer(3 * totalVerticesCount);
        IntBuffer iBuff = GLBuffers.newDirectIntBuffer(totalIndicesCount);

        vertices.forEach(p -> vBuff.put(p.getFloatData()));
        indices.forEach(i -> iBuff.put(i));

        vBuff.rewind();
        iBuff.rewind();

        gl.glBindVertexArray(lineVao[CONVEX]);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, lineVbo[CONVEX]);
        gl.glBufferData(GL4.GL_ARRAY_BUFFER, vBuff.capacity() * Float.BYTES, vBuff, GL4.GL_STATIC_DRAW);
        gl.glVertexAttribPointer(0, 3, GL4.GL_FLOAT, false, 3 * Float.BYTES, 0);
        gl.glEnableVertexAttribArray(0);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, 0);
        gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, lineEbo[CONVEX]);
        gl.glBufferData(GL4.GL_ELEMENT_ARRAY_BUFFER, iBuff.capacity() * Integer.BYTES, iBuff, GL4.GL_STATIC_DRAW);
        gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
        gl.glBindVertexArray(0);
    }

    private void pushConcaveEdgesToGPU(){
        int totalVerticesCount = 0;
        int totalIndicesCount = 0;
        int indexOffset = 0;
        int vertexOffset = 0;
        List<Point> vertices = new ArrayList<>();
        List<Integer> indices = new ArrayList<>();
        for (SphericalPatch cp : concavePatchList){
            cp.lineOffset = indexOffset;
            for (Boundary b : cp.boundaries){
                cp.lineCount += b.lines.size();
                for (Point p : b.vrts){
                    vertices.add(p);
                }
                for (Edge e : b.lines){
                    indices.add(e.v1 + vertexOffset);
                    indices.add(e.v2 + vertexOffset);
                }
                vertexOffset += b.vrts.size();
            }
            indexOffset += cp.lineCount;
        }
        concavePatchesEdgeCount = indices.size() / 2;
        FloatBuffer vBuff = GLBuffers.newDirectFloatBuffer(3 * vertices.size());
        IntBuffer iBuff = GLBuffers.newDirectIntBuffer(indices.size());

        vertices.forEach(p -> vBuff.put(p.getFloatData()));
        indices.forEach(i -> iBuff.put(i));

        vBuff.rewind();
        iBuff.rewind();

        gl.glBindVertexArray(lineVao[CONCAVE]);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, lineVbo[CONCAVE]);
        gl.glBufferData(GL4.GL_ARRAY_BUFFER, vBuff.capacity() * Float.BYTES, vBuff, GL4.GL_STATIC_DRAW);
        gl.glVertexAttribPointer(0, 3, GL4.GL_FLOAT, false, 3 * Float.BYTES, 0);
        gl.glEnableVertexAttribArray(0);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, 0);
        gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, lineEbo[CONCAVE]);
        gl.glBufferData(GL4.GL_ELEMENT_ARRAY_BUFFER, iBuff.capacity() * Integer.BYTES, iBuff, GL4.GL_STATIC_DRAW);
        gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
        gl.glBindVertexArray(0);
    }

    public void sendConvexPatchList(List<SphericalPatch> at){
        convexPatchList = at;
        newAtoms = true;
        afm = new AdvancingFrontMethod();
        atomsMeshed = new boolean[at.size()];
        for (int i = 0; i < atomsMeshed.length; ++i)
            atomsMeshed[i] = false;
    }

    public void sendConcavePatchList(List<SphericalPatch> cp){
        concavePatchList = cp;
        newCPs = true;
        cpAfm = new AdvancingFrontMethod();
        concavePatchesMeshed = new boolean[cp.size()];
        for (int i = 0; i < cp.size(); ++i){
            concavePatchesMeshed[i] = false;
        }
    }

    public void selectConvexPatchesByIDs(List<Integer> ids){
        convexPatchesSelect.clear();
        ids.forEach(i -> { if (i < convexPatchList.size()) { convexPatchesSelect.add(i);}});
    }

    public void selectConcavePatchesByIDs(List<Integer> ids){
        concavePatchesSelect.clear();
        ids.forEach(i -> { if (i < concavePatchList.size()) { concavePatchesSelect.add(i);}});
    }

    public void selectToriPatchesByIDs(List<Integer> ids){
        toriPatchesSelect.clear();
        ids.forEach(i -> { if (i < Surface.rectangles.size()) { toriPatchesSelect.add(i); }});
    }

    public void setup(){
        GLProfile profile = GLProfile.get(GLProfile.GL4);
        GLCapabilities caps = new GLCapabilities(profile);
        window = GLWindow.create(caps);
        window.setSize(800, 600);
        window.setTitle("SES");
        float[] scale = new float[2];
        scale = window.getCurrentSurfaceScale(scale);
        //System.err.println("win scale: " + scale[0] + " " + scale[1]);
        window.addWindowListener(new WindowAdapter() {
            @Override
            public void windowDestroyed(WindowEvent windowEvent) {
                animator.stop();
            }
        });
        window.addWindowListener(new WindowAdapter() {
            @Override
            public void windowMoved(WindowEvent windowEvent) {
                if (controlPanel != null && isPinned.get()){
                    moveControlPanel();
                }
            }
        });
        window.addWindowListener(new WindowAdapter() {
            @Override
            public void windowResized(WindowEvent windowEvent) {
                if (controlPanel != null && isPinned.get()) {
                    moveControlPanel();
                }
            }
        });
        window.addWindowListener(new WindowAdapter() {
            @Override
            public void windowLostFocus(WindowEvent windowEvent) {
                super.windowLostFocus(windowEvent);
                window.confinePointer(false);
                window.setPointerVisible(true);
                viewPanning = false;
                zooming = false;
                captureMouse = false;
            }
        });
        isPinned.addListener(new ChangeListener<Boolean>() {
            @Override
            public void changed(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue) {
                if (newValue){
                    moveControlPanel();
                }
            }
        });
        window.addGLEventListener(this);
        window.addMouseListener(this);
        window.addKeyListener(this);
        window.setVisible(true);
        window.setResizable(true);

        //slerping = true;
        Screen sc = window.getScreen();
        MonitorDevice mon = sc.getPrimaryMonitor();
        //window.setPosition(mon.getViewport().getWidth() / 2 - window.getWidth() / 2, mon.getViewport().getHeight() / 2 - window.getHeight() / 2);
        window.setPosition(0, 0 + window.getInsets().getTopHeight());
        animator = new Animator(window);
        animator.start();

        lastTick = System.currentTimeMillis();

        selectedAtom.addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                lastCameraTarget = convexPatchList.get(newValue.intValue()).sphere.center;
                if (!addToSelection && !removeSelection){
                    convexPatchesSelect.clear();
                    convexPatchesSelect.add(newValue.intValue());
                }
                Platform.runLater(new Runnable() {
                    @Override
                    public void run() {
                        List<String> atomInfo = new ArrayList<>();
                        SphericalPatch a = convexPatchList.get(newValue.intValue());
                        atomInfo.add("Atom radius," + Double.toString(a.sphere.radius));
                        atomInfo.add("Boundary count," + Integer.toString(a.boundaries.size()));
                        for (int i = 0; i < a.boundaries.size(); ++i){
                            atomInfo.add("Boundary " + Integer.toString(i) + "," + Integer.toString(a.boundaries.get(i).arcs.size()));
                        }
                        MainPanelController.cont.updateSelectedAtomInfo(atomInfo);
                    }
                });
            }
        });
        /*selectedAtom.addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                convexPatchesSelect.clear();
                convexPatchesSelect.add(newValue.intValue());
            }
        });*/
        /*selectedToriP.addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                toriPatchesSelect.clear();
                toriPatchesSelect.add(newValue.intValue());
            }
        });*/
        selectedToriP.addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                if (!addToSelection && !removeSelection){
                    toriPatchesSelect.clear();
                    toriPatchesSelect.add(newValue.intValue());
                }
                ToroidalPatch tp = Surface.rectangles.get(newValue.intValue());
                lastCameraTarget = tp.midProbe;
            }
        });
        selectedConcaveP.addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                if (!addToSelection && !removeSelection){
                    concavePatchesSelect.clear();
                    concavePatchesSelect.add(newValue.intValue());
                }
                SphericalPatch cp = concavePatchList.get(newValue.intValue());
                lastCameraTarget = cp.sphere.center;
            }
        });
        /*selectedConcaveP.addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                concavePatchesSelect.clear();
                concavePatchesSelect.add(newValue.intValue());
            }
        });*/
        selectedMeshScaleMat.loadIdentity();
        modelMAT.glLoadIdentity();

        vaos = new ArrayList<>();
        vbos = new ArrayList<>();
        ebos = new ArrayList<>();
        MainWindow.mainWindow = this;
    }

    public void showProbe(boolean v){
        renderProbe = v;
    }

    private void moveControlPanel(){
        MainPanel.pinnedToView = false;
        controlPanel.setX(window.getX() + window.getWidth() - window.getInsets().getRightWidth());
        controlPanel.setY(window.getY() - window.getInsets().getTopHeight());
        MainPanel.pinnedToView = true;
    }

    public void setWindowPosition(int x, int y){
        this.window.setPosition(x - this.window.getWidth(), y + this.window.getInsets().getTopHeight());
    }

    @Override
    public void init(GLAutoDrawable glAutoDrawable) {
        gl = GLContext.getCurrentGL().getGL4();
        mainProgram = GLUtil.createShaderProgram("./resources/shaders/main.vert", "./resources/shaders/main.frag");
        uniMeshColorLoc = gl.glGetUniformLocation(mainProgram, "col");
        uniProjMatLoc = gl.glGetUniformLocation(mainProgram, "projMat");
        uniViewMatLoc = gl.glGetUniformLocation(mainProgram, "viewMat");
        uniModelMatLoc = gl.glGetUniformLocation(mainProgram, "modelMat");
        uniAlphaLoc = gl.glGetUniformLocation(mainProgram, "alpha");
        uniNormalColorLoc = gl.glGetUniformLocation(mainProgram, "normalColor");
        uniSelectedColorLoc = gl.glGetUniformLocation(mainProgram, "selectedColor");
        uniSelectedMeshStartLoc = gl.glGetUniformLocation(mainProgram, "selectedMeshStart");
        uniSelectedMeshEndLoc = gl.glGetUniformLocation(mainProgram, "selectedMeshEnd");
        uniAmbientStrengthLoc = gl.glGetUniformLocation(mainProgram, "ambientStrength");
        uniSelectedMeshCountLoc = gl.glGetUniformLocation(mainProgram, "selectedMeshCount");
        uniMvInverseLoc = gl.glGetUniformLocation(mainProgram, "mvInverse");
        //rectProgram = Util.createShaderProgram("rect.vert", "rect.frag");
        selProgram = GLUtil.createShaderProgram("./resources/shaders/sel2.vert", "./resources/shaders/sel.frag");
        uniEnd = gl.glGetUniformLocation(selProgram, "end");
        uniStart = gl.glGetUniformLocation(selProgram, "start");
        uniGlobalOffset = gl.glGetUniformLocation(selProgram, "globalOffset");
        uniVertexOffsets = gl.glGetUniformLocation(selProgram, "u_offset_Tex");

        right = VectorUtil.crossVec3(right, direction, up);
        up = VectorUtil.normalizeVec3(up);
        right = VectorUtil.normalizeVec3(right);
        direction = VectorUtil.normalizeVec3(direction);
        //gl.glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
        gl.glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
        //gl.glClearColor(0.45f, 0.45f, 0.45f, 1.0f);
        gl.glEnable(GL.GL_DEPTH_TEST);
        gl.glEnable(GL.GL_LINE_SMOOTH);
        gl.glEnable(GL.GL_CULL_FACE);
        gl.glLineWidth(0.5f);
        gl.glCullFace(GL.GL_BACK);
        float aspect = 800 / (float)600;
        pMat = GLUtil.perspective(60.f, aspect, 0.1f, 100.f);
        lastTick = System.currentTimeMillis();
        List<Point> vrts = new ArrayList<>();
        for (int i = 0; i < data.length; i += 3){
            vrts.add(new Point(data[i], data[i + 1], data[i + 2]));
        }
        List<Edge> lines = new ArrayList<>();
        for (int i = 0; i < indices.length; i += 2){
            lines.add(new Edge(indices[i], indices[i + 1]));
        }
        mouseLocation = new Point(0, 0, 0);
        gl.glGenVertexArrays(1, mouseSelectVao, 0);
        gl.glGenBuffers(1, mouseSelectVbo, 0);
        FloatBuffer bff = GLBuffers.newDirectFloatBuffer(6);
        bff.put(new float[]{0.f, 0.f, 0.f});
        bff.rewind();
        gl.glBindVertexArray(mouseSelectVao[0]);
        gl.glBindBuffer(GL.GL_ARRAY_BUFFER, mouseSelectVbo[0]);
        gl.glBufferData(GL.GL_ARRAY_BUFFER, bff.capacity() * Float.BYTES, bff, GL.GL_DYNAMIC_DRAW);
        gl.glVertexAttribPointer(0, 3, GL.GL_FLOAT, false, 3 * Float.BYTES, 0);
        gl.glEnableVertexAttribArray(0);
        gl.glBindBuffer(GL.GL_ARRAY_BUFFER, 0);
        gl.glBindVertexArray(0);

        gl.glGenFramebuffers(1, fbo, 0);
        gl.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo[0]);
        gl.glGenRenderbuffers(1, rbCol, 0);
        gl.glGenRenderbuffers(1, rbDep, 0);
        gl.glBindRenderbuffer(GL.GL_RENDERBUFFER, rbCol[0]);
        gl.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL4.GL_R32I, window.getWidth(), window.getHeight());
        gl.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL4.GL_COLOR_ATTACHMENT0, GL4.GL_RENDERBUFFER, rbCol[0]);
        gl.glBindRenderbuffer(GL.GL_RENDERBUFFER, rbDep[0]);
        gl.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL4.GL_DEPTH_COMPONENT24, window.getWidth(), window.getHeight());
        gl.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL4.GL_DEPTH_ATTACHMENT, GL4.GL_RENDERBUFFER, rbDep[0]);
        gl.glBindRenderbuffer(GL.GL_RENDERBUFFER, 0);
        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, 0);

        int[] bobjects = GLUtil.loadSphere("./resources/misc/sphere3.obj", gl);
        this.probeVao[0] = bobjects[0];
        this.probeVbo[0] = bobjects[1];
        this.probeFaceCount = bobjects[2];
        this.probeScaleT.glLoadIdentity();
        float r = (float)Double.longBitsToDouble(Surface.probeRadius.get());
        //this.probeScaleT.glScalef(r, r, r);
        this.probeScaleT.glPushMatrix();
        camDir = new Quaternion(direction[0] ,direction[1], direction[2], 0.f);
        camDir.normalize();
        camTar = new Quaternion((float) Surface.centerOfgravity.getX() - cameraPos[0], (float) Surface.centerOfgravity.getY() - cameraPos[1], (float) Surface.centerOfgravity.getZ() - cameraPos[2], 0.f);
        camTar.normalize();
        slerping = true;
        slerpParam = 0;
        cameraTarget = new Point3D(0.f, 0.f, -2.f);
        lastCameraTarget = new Point(cameraTarget.getX(), cameraTarget.getY(), cameraTarget.getZ());
        lightPos = new Point3D(cameraPos[0], cameraPos[1], cameraPos[2]);
        right = new float[]{1.0f, 0.f, 0.f};
        up = VectorUtil.crossVec3(up, right, direction);
    }

    public void sendNewData(List<Point> vrts, List<Edge> lines){
        if (buffersInitialized){
            gl.glDeleteVertexArrays(1, vao, 0);
            gl.glDeleteBuffers(1, vbo, 0);
            gl.glDeleteBuffers(1, ebo, 0);
        }
        FloatBuffer vrtsBuffer = GLBuffers.newDirectFloatBuffer(3 * vrts.size());
        Point p = vrts.get((vrts.size() + 1) / 2 - 1);

        vrts.stream().forEach(f -> {for (int i = 0; i < 3; ++i){
            if (i == 0){
                vrtsBuffer.put((float)(f.x - p.x));
            } else if (i == 1){
                vrtsBuffer.put((float)(f.y - p.y));
            } else {
                vrtsBuffer.put((float)(f.z - p.z));
            }
        }});
        vrtsBuffer.rewind();
        /*while (vrtsBuffer.hasRemaining()){
            System.out.println(vrtsBuffer.get());
        }

        for (Edge l : lines){
            System.out.println(l.toOBJString(0));
        }*/

        numOfVrts = vrts.size();
        IntBuffer indBuffer = GLBuffers.newDirectIntBuffer(2 * lines.size());
        numOfIndices = indBuffer.capacity();
        lines.forEach(l -> {indBuffer.put(l.v1); indBuffer.put(l.v2);});
        indBuffer.rewind();
        /*while (indBuffer.hasRemaining()){
            System.out.println(indBuffer.get());
        }*/
        vrtsBuffer.rewind();
        indBuffer.rewind();
        GL4 gl2 = GLContext.getCurrentGL().getGL4();
        gl2.glGenVertexArrays(1, vao, 0);
        gl2.glGenBuffers(1, vbo, 0);
        gl2.glGenBuffers(1, ebo, 0);
        gl2.glBindVertexArray(vao[0]);

        gl2.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo[0]);
        gl2.glBufferData(GL.GL_ARRAY_BUFFER, vrtsBuffer.capacity() * Float.BYTES, vrtsBuffer, GL.GL_STATIC_DRAW);
        gl2.glVertexAttribPointer(0, 3, GL.GL_FLOAT, false, 3 * Float.BYTES, 0);
        gl2.glEnableVertexAttribArray(0);
        gl2.glBindBuffer(GL.GL_ARRAY_BUFFER, 0);
        gl2.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo[0]);
        gl2.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, indBuffer.capacity() * Integer.BYTES, indBuffer, GL.GL_STATIC_DRAW);
        gl2.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0);
        gl2.glBindVertexArray(0);
        buffersInitialized = true;
        System.out.println("COMPLETE");
    }

    @Override
    public void dispose(GLAutoDrawable glAutoDrawable) {
        stoppedRendering.set(true);
        freeGLResources();
        animator.stop();
    }

    private void deleteRenderBuffers(){
        gl.glDeleteRenderbuffers(1, rbCol, 0);
        gl.glDeleteRenderbuffers(1, rbDep, 0);
    }

    private void constructNewRenderBuffers(){
        if (renderBuffersInit){
            deleteRenderBuffers();
        }
        gl.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo[0]);
        gl.glGenRenderbuffers(1, rbCol, 0);
        gl.glGenRenderbuffers(1, rbDep, 0);
        gl.glBindRenderbuffer(GL.GL_RENDERBUFFER, rbCol[0]);
        gl.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL4.GL_R32I, window.getWidth(), window.getHeight());
        gl.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL4.GL_COLOR_ATTACHMENT0, GL4.GL_RENDERBUFFER, rbCol[0]);
        gl.glBindRenderbuffer(GL.GL_RENDERBUFFER, rbDep[0]);
        gl.glRenderbufferStorage(GL.GL_RENDERBUFFER, GL4.GL_DEPTH_COMPONENT24, window.getWidth(), window.getHeight());
        gl.glFramebufferRenderbuffer(GL.GL_FRAMEBUFFER, GL4.GL_DEPTH_ATTACHMENT, GL4.GL_RENDERBUFFER, rbDep[0]);

        if (gl.glCheckFramebufferStatus(GL4.GL_FRAMEBUFFER) == GL4.GL_FRAMEBUFFER_COMPLETE){
            renderBuffersInit = true;
        } else {
            renderBuffersInit = false;
            deleteRenderBuffers();
        }
        gl.glBindRenderbuffer(GL.GL_RENDERBUFFER, 0);
        gl.glBindFramebuffer(GL4.GL_FRAMEBUFFER, 0);

    }
    private void renderProbeAndNeighborProbes(){
        if (renderProbe) {
            gl.glUseProgram(mainProgram);
            gl.glEnable(GL.GL_BLEND);
            gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);
            ///gl.glUniform4f(atomColor, 1.f, 1.f, 1.f, 0.4f);
            gl.glUniform3f(uniMeshColorLoc, 1.f, 1.f, 1.f);
            gl.glUniform1f(uniAlphaLoc, probeAlpha);
            if (linkNeighbors.size() == 0) {
                /*ConcavePatch cp = concavePatchList.get(selectedConcaveP.get());
                probeScaleT.glPushMatrix();
                float r = (float) Double.longBitsToDouble(Main.probeRadius.get());

                float[] center = cp.probe.getFloatData();
                //probeScaleT.glMultMatrixf(modelMAT.glGetMatrixf());
                probeScaleT.glTranslatef(center[0], center[1], center[2]);
                probeScaleT.glScalef(r, r, r);
                gl.glUniformMatrix4fv(uniModelMatLoc, 1, false, probeScaleT.glGetMatrixf());
                gl.glBindVertexArray(probeVao[0]);
                gl.glDrawArrays(GL.GL_TRIANGLES, 0, 3 * probeFaceCount);
                gl.glBindVertexArray(0);
                probeScaleT.glPopMatrix();*/
                float r = (float)SesConfig.probeRadius;
                for (Integer i : concavePatchesSelect){
                    SphericalPatch cp2 = concavePatchList.get(i);
                    float[] c2 = cp2.sphere.center.getFloatData();
                    probeScaleT.glPushMatrix();
                    probeScaleT.glTranslatef(c2[0], c2[1], c2[2]);
                    probeScaleT.glScalef(r, r, r);
                    gl.glUniformMatrix4fv(uniModelMatLoc, 1, false, probeScaleT.glGetMatrixf());
                    gl.glBindVertexArray(probeVao[0]);
                    gl.glDrawArrays(GL4.GL_TRIANGLES, 0, 3 * probeFaceCount);
                    gl.glBindVertexArray(0);
                    probeScaleT.glPopMatrix();
                }

            } else {
                for (Integer i : linkNeighbors) {
                    SphericalPatch cp = concavePatchList.get(i);
                    probeScaleT.glPushMatrix();
                    float r = (float) Double.longBitsToDouble(Surface.probeRadius.get());

                    float[] center = cp.sphere.center.getFloatData();
                    probeScaleT.glTranslatef(center[0], center[1], center[2]);
                    probeScaleT.glScalef(r, r, r);
                    gl.glUniformMatrix4fv(uniModelMatLoc, 1, false, probeScaleT.glGetMatrixf());
                    gl.glBindVertexArray(probeVao[0]);
                    gl.glDrawArrays(GL.GL_TRIANGLES, 0, 3 * probeFaceCount);
                    gl.glBindVertexArray(0);
                    probeScaleT.glPopMatrix();
                }
            }
            gl.glDisable(GL.GL_BLEND);
        }
    }
    @Override
    public void display(GLAutoDrawable glAutoDrawable) {
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        if (cullFaces){
            gl.glEnable(GL.GL_CULL_FACE);
        } else {
            gl.glDisable(GL.GL_CULL_FACE);
        }
        if (drawModeUpdate){
            gl.glPolygonMode(GL4.GL_FRONT_AND_BACK, (drawFaces) ? GL4.GL_FILL : GL4.GL_LINE);
            drawModeUpdate = false;
        }
        updateCamera();
        if (isNew){
            this.sendNewData(vrts, lines);
            isNew = false;
        }
        /*if (newAtoms){
            this.sendConvexPatches2GPU();
            newAtoms = false;
        }
        if (newCPs){
            sendConcavePathes2GPU();
        }*/

        //gl.glPolygonMode(GL.GL_FRONT_AND_BACK, (drawFaces) ? GL4.GL_FILL : GL4.GL_LINE);
        if (update2.get()){
            SphericalPatch a = convexPatchList.get(selectedAtom.get());
            //gl.glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
            gl.glBindVertexArray(a.vao[1]);
            if (updaVrts.size() > 0){
                gl.glBindBuffer(GL.GL_ARRAY_BUFFER, a.vbo[1]);
                FloatBuffer vrts = GLBuffers.newDirectFloatBuffer(updaVrts.size() * 6);
                for (Point p : updaVrts){
                    vrts.put(p.getFloatData());
                    Vector n = Point.subtractPoints(p, a.sphere.center).makeUnit();
                    vrts.put(n.getFloatData());
                }
                vrts.rewind();
                gl.glBufferSubData(GL.GL_ARRAY_BUFFER, a.usedVrtsBuffer, vrts.capacity() * Float.BYTES, vrts);
                gl.glBindBuffer(GL.GL_ARRAY_BUFFER, 0);
                a.usedVrtsBuffer += updaVrts.size() * 6 * Float.BYTES;
                a.vrtsCount += updaVrts.size();
                System.out.println("USED Vrt: " + a.usedVrtsBuffer);
            }
            IntBuffer indx = GLBuffers.newDirectIntBuffer(3 * updaFaces.size());
            /*for (Edge e : updaLines){
                indx.put(e.v1);
                indx.put(e.v2);
            }*/
            for (Face f : updaFaces){
                indx.put(f.a);
                indx.put(f.b);
                indx.put(f.c);
            }
            indx.rewind();
            gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, a.ebo[1]);
            gl.glBufferSubData(GL.GL_ELEMENT_ARRAY_BUFFER, a.usedFaceBuffer, indx.capacity() * Integer.BYTES, indx);
            gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0);
            /*a.usedIndBuffer += 2 * updaLines.size() * Integer.BYTES;
            a.lineCount += 2 * updaLines.size();*/
            a.faceCount += updaFaces.size();
            a.usedFaceBuffer += 3 * updaFaces.size() * Integer.BYTES;
            gl.glBindVertexArray(0);
            update = false;
            //update2.set(false);
            update2.getAndSet(false);
        }
        if (cpUpdate.get()){
            SphericalPatch c = concavePatchList.get(concavePatchesSelect.get(0));
            gl.glBindVertexArray(c.vao[1]);
            if (cpVrts.size() > 0){
                gl.glBindBuffer(GL.GL_ARRAY_BUFFER, c.vbo[1]);
                FloatBuffer newVrts = GLBuffers.newDirectFloatBuffer(cpVrts.size() * 3 * 2);
                for (Point p : cpVrts){
                    newVrts.put(p.getFloatData());
                    Vector n = Point.subtractPoints(c.boundaries.get(0).patch.sphere.center, p).makeUnit();
                    newVrts.put(n.getFloatData());
                }
                newVrts.rewind();
                gl.glBufferSubData(GL.GL_ARRAY_BUFFER, c.usedVrtsBuffer, newVrts.capacity() * Float.BYTES, newVrts);
                c.usedVrtsBuffer += cpVrts.size() * 3 * Float.BYTES * 2;
                gl.glBindBuffer(GL.GL_ARRAY_BUFFER, 0);
            }
            IntBuffer indBuffer = GLBuffers.newDirectIntBuffer(cpFaces.size() * 3);
            for (Face f : cpFaces){
                indBuffer.put(f.a);
                indBuffer.put(f.b);
                indBuffer.put(f.c);
            }
            indBuffer.rewind();
            gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, c.ebo[1]);
            gl.glBufferSubData(GL.GL_ELEMENT_ARRAY_BUFFER, c.usedIndBuffer, indBuffer.capacity() * Integer.BYTES, indBuffer);
            gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0);
            gl.glBindVertexArray(0);
            c.faceCount += cpFaces.size();
            c.usedIndBuffer += 3 * cpFaces.size() * Integer.BYTES;
            cpUpdate.set(false);
        }
        if (!stopRendering.get()){
            if (mouseSelect && moved){
                moved = false;
                gl.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo[0]);
                IntBuffer val = GLBuffers.newDirectIntBuffer(1);
                gl.glReadPixels((int)mouseLocation.x, window.getHeight() - 1 - (int)mouseLocation.y, 1, 1, GL4.GL_RED_INTEGER, GL4.GL_INT, val);
                hoverAtom = val.get();
                //System.out.println("id: " + hoverAtom);
                gl.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0);
            }
            PMVMatrix modelMat = new PMVMatrix();
            modelMat.glLoadIdentity();
            //modelMat.glTranslatef(modelX, modelY, modelZ);
            gl.glUseProgram(mainProgram);
            /*PMVMatrix projection = new PMVMatrix();
            projection.glLoadIdentity();
            projection.gluPerspective((float)Math.toRadians(45), 4.0f/3.f, 0.01f, 100.f);
            PMVMatrix view = new PMVMatrix();
            view.glLoadIdentity();
            view.gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2], cameraPos[0] + direction[0], cameraPos[1] + direction[1], cameraPos[2] + direction[2], up[0],
            up[1], up[2]);
            //projection.glMultMatrixf(view.glGetMatrixf());
            view.glMultMatrixf(projection.glGetMatrixf());*/
            Matrix3D vMat = new Matrix3D();
            vMat.translate(-cameraPos[0], -cameraPos[1], -cameraPos[2]);
            Matrix3D mMat = new Matrix3D();
            mMat.translate(modelX, modelY,modelZ);
            Matrix3D mvMat = new Matrix3D();
            mvMat.concatenate(vMat);
            mvMat.concatenate(mMat);

            PMVMatrix look = new PMVMatrix();
            look.glLoadIdentity();
            /*Vector3D target = new Vector3D(cameraTarget.getX() - cameraPos[0],
                    cameraTarget.getY() - cameraPos[1],
                    cameraTarget.getZ() - cameraPos[2]);
            Vector3D yAx = new Vector3D(0.f, 1.f, 0.f);
            Vector3D right = target.cross(yAx);
            Vector3D up = right.cross(target);
            look.gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],
                    (float)cameraTarget.getX(), (float)cameraTarget.getY(), (float)cameraTarget.getZ(),
                    (float)up.getX(), (float)up.getY(), (float)up.getZ());
            PMVMatrix view = new PMVMatrix();
            view.glTranslatef(-cameraPos[0], -cameraPos[1], -cameraPos[2]);*/
            look.gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],
                    cameraPos[0] + direction[0], cameraPos[1] + direction[1], + cameraPos[2] + direction[2],
                    up[0], up[1], up[2]);

            //int mv_loc = gl.glGetUniformLocation(mainProgram, "mv_matrix");
            //int view_loc = gl.glGetUniformLocation(mainProgram, "viewMat");
            //int proj_loc = gl.glGetUniformLocation(mainProgram, "proj_matrix");
            //int uniModelMatLoc = gl.glGetUniformLocation(mainProgram, "modelMat");
            //int uniMeshColorLoc = gl.glGetUniformLocation(mainProgram, "col");
            int lightColor_loc = gl.glGetUniformLocation(mainProgram, "lightColor");
            int lightPos_loc = gl.glGetUniformLocation(mainProgram, "lightPos");
            int alpha_loc = gl.glGetUniformLocation(mainProgram, "alpha");
            Matrix4 mat = new Matrix4();
            //gl.glBindVertexArray(vao[0]);
            gl.glUniformMatrix4fv(uniProjMatLoc, 1, false, pMat.getFloatValues(), 0);
            gl.glUniformMatrix4fv(uniViewMatLoc, 1, false, look.glGetMatrixf());
            gl.glUniformMatrix4fv(uniModelMatLoc, 1, false, modelMAT.glGetMatrixf());
            gl.glUniformMatrix4fv(uniMvInverseLoc, 1, false, modelMAT.glGetMvitMatrixf());
            gl.glUniform3f(lightColor_loc, 1.f, 1.f, 1.f);
            Point p = new Point(lightPos.getX(), lightPos.getY(), lightPos.getZ());
            //float[] light = new float[3];
            //light =
            gl.glUniform3f(lightPos_loc, (float)lightPos.getX(), (float)lightPos.getY(), (float)lightPos.getZ());
            gl.glUniform1f(uniAlphaLoc, 1.f);
            //gl.glClearColor(0.45f, 0.45f, 0.45f, 1.0f);
            //gl.glDrawArrays(GL.GL_LINES, 0, numOfVrts);
            //renderConvexPatches();
            drawConvex();
            drawConcave();
            drawTori();
            //renderConcavePatches();
            //renderToriPatches();
            renderProbeAndNeighborProbes();
        } else {
            stoppedRendering.set(true);
        }
            gl.glUseProgram(0);
            gl.glBindVertexArray(0);

        deltaTime = (float)(System.currentTimeMillis() - lastTick);
        /*if (System.currentTimeMillis() > fpsLastTick + 250) {
            String[] s = window.getTitle().split(" FPS:");
            window.setTitle(s[0] + " FPS: " + 1000.f / deltaTime);
            fpsLastTick = System.currentTimeMillis();
        }*/
        angle += deltaTime * 0.5f;
        //System.out.println(deltaTime);
        lastTick = System.currentTimeMillis();
    }

    @Override
    public void reshape(GLAutoDrawable glAutoDrawable, int i, int i1, int i2, int i3) {
        float aspect = (float)i2 / (float)i3;
        pMat = GLUtil.perspective(60.f, aspect, 0.1f, 100.f);
        constructNewRenderBuffers();
        updateCamera();
    }

    private boolean cameraMoving = false;
    private int cforward = 0;
    private int cright = 0;

    private void updateCamera(){
        cameraPos[0] += direction[0] * cforward * speed * deltaTime;
        cameraPos[1] += direction[1] * cforward * speed * deltaTime;
        cameraPos[2] += direction[2] * cforward * speed * deltaTime;

        cameraPos[0] += right[0] * cright * speed * deltaTime;
        cameraPos[1] += right[1] * cright * speed * deltaTime;
        cameraPos[2] += right[2] * cright * speed * deltaTime;

        lightPos.setX(cameraPos[0]);
        lightPos.setY(cameraPos[1]);
        lightPos.setZ(cameraPos[2]);
        if (slerping && slerpParam < 1.f){

            //camDir = camDir.setSlerp(camDir, camTar, 0.01f);
            //System.out.println("slerping");
            if (camDir.dot(camTar) < 0.0f){
                camDir.scale(-1.f);
            }
            Quaternion slerpq = new Quaternion(0.f, 0.f, 0.f, 0.f);
            slerpq.setSlerp(camDir, camTar, slerpParam);
            slerpParam += 0.01f * deltaTime;
            direction[0] = slerpq.getX();
            direction[1] = slerpq.getY();
            direction[2] = slerpq.getZ();
            direction = VectorUtil.normalizeVec3(direction);
            //float[] right = VectorUtil.crossVec3(null, direction, new float[]{0.f, 1.f, 0.f});
            right = VectorUtil.crossVec3(right, new float[]{0.f, 1.f, 0.f}, direction);
            up = VectorUtil.crossVec3(up, right, direction);
        } else {
            slerping = false;
            slerpParam = 0.f;
        }
        if (mouseSelect){
            if (convexPatchesFaceCount == 0 || concavePatchesFaceCount == 0 || toriPatchesFaceCount == 0){
                return;
            }
            gl.glUseProgram(selProgram);
            if (!selectInitialized){
                //Integer[] offsets = new Integer[convexPatches.size() + concavePatchList.size() + Main.rectangles.size()];
                IntBuffer boffsets = GLBuffers.newDirectIntBuffer(convexPatchList.size() + concavePatchList.size() + Surface.rectangles.size());
                int accumulator = 0;
                for (SphericalPatch a : convexPatchList){
                    boffsets.put(accumulator);
                    accumulator += a.vertices.size();
                }
                convexVerticesCount = accumulator;
                for (SphericalPatch cp : concavePatchList){
                    boffsets.put(accumulator);
                    accumulator += cp.vertices.size();
                    concaveVerticesCount += cp.vertices.size();
                }
                for (ToroidalPatch tp : Surface.rectangles){
                    boffsets.put(accumulator);
                    accumulator += tp.vrts.size() / 2;
                    toriVerticesCount += tp.vrts.size() / 2;
                }
                //toriVerticesCount = accumulator;
                boffsets.rewind();
                //TextureData tdata = new TextureData(GLProfile.GL4, GL4.GL_R32I, boffsets.capacity(), 1, 0, )
                gl.glGenBuffers(1, tbo, 0);
                gl.glBindBuffer(GL4.GL_TEXTURE_BUFFER, tbo[0]);
                gl.glBufferData(GL4.GL_TEXTURE_BUFFER, boffsets.capacity() * Integer.BYTES, boffsets, GL4.GL_STATIC_DRAW);
                gl.glGenTextures(1, tboTex, 0);
                gl.glBindBuffer(GL4.GL_TEXTURE_BUFFER, 0);
                //gl.glUniform1iv(uniVertexOffsets, 0, boffsets);
                selectInitialized = true;
            }
            gl.glBindFramebuffer(GL.GL_FRAMEBUFFER, fbo[0]);
            gl.glEnable(GL.GL_DEPTH_TEST);
            gl.glClearColor(-1.f, -1.f, -1.f, -1.f);
            gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);

            gl.glActiveTexture(GL4.GL_TEXTURE0);
            gl.glBindTexture(GL4.GL_TEXTURE_BUFFER, tboTex[0]);
            gl.glTexBuffer(GL4.GL_TEXTURE_BUFFER, GL4.GL_R32I, tbo[0]);
            gl.glUniform1i(uniVertexOffsets, 0);

            //gl.glUseProgram(selProgram);
            int viewLoc = gl.glGetUniformLocation(selProgram, "viewMat");
            int projLoc = gl.glGetUniformLocation(selProgram, "proj_matrix");
            int modelLoc = gl.glGetUniformLocation(selProgram, "modelMat");
            //int atomIdLoc = gl.glGetUniformLocation(selProgram, "atomID");
            PMVMatrix look = new PMVMatrix();
            look.glLoadIdentity();
            look.gluLookAt(cameraPos[0], cameraPos[1], cameraPos[2],
                    cameraPos[0] + direction[0], cameraPos[1] + direction[1], + cameraPos[2] + direction[2],
                    up[0], up[1], up[2]);
            PMVMatrix modelMat = new PMVMatrix();
            modelMat.glLoadIdentity();

            gl.glUniformMatrix4fv(projLoc, 1, false, pMat.getFloatValues(), 0);
            gl.glUniformMatrix4fv(viewLoc, 1, false, look.glGetMatrixf());
            gl.glUniformMatrix4fv(modelLoc, 1, false, modelMAT.glGetMatrixf());

            gl.glUniform1i(uniStart, 0);
            gl.glUniform1i(uniEnd, convexPatchList.size());
            gl.glUniform1i(uniGlobalOffset, 0);
            gl.glBindVertexArray(meshVao[CONVEX]);
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[CONVEX]);
            gl.glDrawElements(GL4.GL_TRIANGLES, 3 * convexPatchesFaceCount, GL4.GL_UNSIGNED_INT, 0);

            gl.glFrontFace(GL4.GL_CW);
            gl.glUniform1i(uniGlobalOffset, convexVerticesCount);
            gl.glUniform1i(uniStart, convexPatchList.size());
            gl.glUniform1i(uniEnd, convexPatchList.size() + concavePatchList.size());
            gl.glBindVertexArray(meshVao[CONCAVE]);
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[CONCAVE]);
            gl.glDrawElements(GL4.GL_TRIANGLES, 3 * concavePatchesFaceCount, GL4.GL_UNSIGNED_INT, 0);
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
            gl.glFrontFace(GL4.GL_CCW);

            gl.glUniform1i(uniStart, concavePatchList.size() + convexPatchList.size());
            gl.glUniform1i(uniEnd, concavePatchList.size() + convexPatchList.size() + Surface.rectangles.size());
            gl.glUniform1i(uniGlobalOffset, convexVerticesCount + concaveVerticesCount);
            gl.glBindVertexArray(meshVao[TORUS]);
            gl.glDrawArrays(GL4.GL_TRIANGLES, 0, 3 * toriPatchesFaceCount);
            gl.glBindVertexArray(0);
            /*for (int i = 0; i < convexPatches.size(); ++i){
                Atom a = convexPatches.get(i);
                gl.glUniform1i(atomIdLoc, i);
                gl.glBindVertexArray(a.vao[1]);
                gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, a.ebo[1]);
                gl.glDrawElements(GL.GL_TRIANGLES, 3 * a.faceCount, GL.GL_UNSIGNED_INT, 0);
                gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0);
                gl.glBindVertexArray(0);
            }
            gl.glFrontFace(GL.GL_CW);
            for (int i = 0; i < concavePatchList.size(); ++i){
                ConcavePatch cp = concavePatchList.get(i);
                gl.glUniform1i(atomIdLoc, i + convexPatches.size());
                gl.glBindVertexArray(cp.vao[1]);
                gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, cp.ebo[1]);
                gl.glDrawElements(GL.GL_TRIANGLES, 3 * cp.faceCount, GL.GL_UNSIGNED_INT, 0);
                gl.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0);
                gl.glBindVertexArray(0);
            }
            gl.glFrontFace(GL.GL_CCW);
            for (int i = 0; i < Main.rectangles.size(); ++i){
                RollingPatch rp = Main.rectangles.get(i);
                gl.glUniform1i(atomIdLoc, i + convexPatches.size() + concavePatchList.size());
                gl.glBindVertexArray(rp.vao[0]);
                gl.glDrawArrays(GL.GL_TRIANGLES, 0, rp.vrts.size());
                gl.glBindVertexArray(0);
            }*/
            gl.glBindFramebuffer(GL.GL_FRAMEBUFFER, 0);
            gl.glUseProgram(mainProgram);
            //gl.glClearColor(0.45f, 0.45f, 0.45f, 1.0f);
            gl.glClearColor(1.f, 1.f, 1.f, 1.f);
        }
    }

    private void meshConvexPatches(int start, int end, boolean waitForOthers) {
        stopRendering.set(true);
        List<Point> verts = new ArrayList<>();
        List<Face> faces = new ArrayList<>();
        AdvancingFrontMethod afm = new AdvancingFrontMethod();
        long startTime = System.currentTimeMillis();
        for (int i = start; i < end; ++i) {
            SphericalPatch a = convexPatchList.get(i);
            if (!a.valid){
                MeshRefinement.refinement.enqueue(a);
                continue;
            }
            if (!atomsMeshed[i]) {
                //selectedAtom.set(i);
                //List<Point> noveBody = new ArrayList<>();
                long time = 0;
                //System.out.println("Meshing atom " + i);
                if (a.boundaries.size() > 0) {
                    afm._initializeConvexAFM(a, Math.toRadians(SesConfig.minAlpha), SesConfig.distTolerance, Surface.maxEdgeLen * (Math.sqrt(3) / 2.f));
                    //updaFaces = afm.soho(a.convexPatchBoundaries.get(0), Math.toRadians(75), 0.2, 0.3 * (Math.sqrt(3) / 2.f), false, noveBody);
                        /*do {
                            overallTime += System.currentTimeMillis() - time;
                            updaVrts.clear();
                            updaFaces.clear();

                            afm.mesh(noveBody, updaFaces, false);
                            //System.out.println("Done meshing atom " + i);
                            updaVrts = noveBody;
                            //update =
                            //update2.set(true);
                            atomsMeshed[i] = true;
                            time = System.currentTimeMillis();
                            /*while (update2.get()) {
                                //System.out.println("Waiting for gl");
                            }
                        } while (!afm.atomComplete);*/
                    verts.clear();
                    faces.clear();
                    //time = System.currentTimeMillis();
                    do {
                        afm._mesh2();
                    } while (!afm.atomComplete);
                    if (afm.volpe){
                        System.out.println("convex " + i + " looped");
                    }
                    //meshTime += (System.currentTimeMillis() - time);
                    atomsMeshed[i] = true;
                    //update2.getAndSet(true);
                    //trianglesCount += updaFaces.size();
                    //while (update2.get()) {
                        //System.out.println(".");
                    //}
                }
                MeshRefinement.refinement.enqueue(a);
            }
        }
        long endTime = System.currentTimeMillis();
        System.out.println("Meshed in " + (endTime - startTime) + " ms");
        convexMeshThreadsCounter.getAndIncrement();
        if (waitForOthers && false){
            while (convexMeshThreadsCounter.get() < 4){}
            GLRunnable task = new GLRunnable() {
                @Override
                public boolean run(GLAutoDrawable glAutoDrawable) {
                    pushConvexPatchesToGPU();
                    convexMeshInitialized = true;
                    return true;
                }
            };
            window.invoke(false, task);
        } else {
            //convexPushData2GPU.set(true);

        }
    }

    private void drawConvex(){
        gl.glUniform3fv(uniNormalColorLoc, 1, convexPatchCol, 0);
        gl.glUniform3fv(uniSelectedColorLoc, 1, selectedPatchCol, 0);
        if (convexMeshInitialized) {
            if (selectedExclusiveRender){
                if (convexPatchesSelect.size() > 0) {
                    gl.glUniform1i(uniSelectedMeshCountLoc, 0);
                    gl.glUniform3fv(uniNormalColorLoc, 1, convexPatchCol, 0);
                    gl.glBindVertexArray(meshVao[CONVEX]);
                    gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[CONVEX]);
                    for (Integer i : convexPatchesSelect) {
                        SphericalPatch a = convexPatchList.get(i);
                        int off = (convexFaceCountShow > a.faces.size()) ? a.faces.size() : convexFaceCountShow;
                        gl.glDrawElements(GL4.GL_TRIANGLES, 3 * a.faces.size() - 3 * off, GL4.GL_UNSIGNED_INT, a.eboOffset * Integer.BYTES);
                    }
                }
            } else {
                gl.glUniform1iv(uniSelectedMeshStartLoc, 20, zeroes);
                gl.glUniform1iv(uniSelectedMeshEndLoc, 20, zeroes);
                IntBuffer start = GLBuffers.newDirectIntBuffer(convexPatchesSelect.size());
                IntBuffer end = GLBuffers.newDirectIntBuffer(convexPatchesSelect.size());
                for (Integer i : convexPatchesSelect) {
                    SphericalPatch a = convexPatchList.get(i);
                    start.put(a.vboOffset);
                    end.put(a.vboOffset + a.vertices.size());
                }
                if (convexPatchesSelect.size() == 0) {
                    start = GLBuffers.newDirectIntBuffer(20);
                    for (int i = 0; i < 20; ++i) {
                        start.put(-1);
                    }
                }
                start.rewind();
                end.rewind();
                gl.glUniform1i(uniSelectedMeshCountLoc, convexPatchesSelect.size());
                gl.glUniform1iv(uniSelectedMeshStartLoc, start.capacity(), start);
                gl.glUniform1iv(uniSelectedMeshEndLoc, end.capacity(), end);
                gl.glUniform1f(uniAmbientStrengthLoc, 0.5f);
                gl.glUniform3fv(uniMeshColorLoc, 1, convexPatchCol, 0);
                gl.glBindVertexArray(meshVao[CONVEX]);
                gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[CONVEX]);
                gl.glDrawElements(GL4.GL_TRIANGLES, 3 * convexPatchesFaceCount, GL4.GL_UNSIGNED_INT, 0);
                gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
                gl.glBindVertexArray(0);
            }
        }
        if (renderLines) {
            gl.glUniform1f(uniAmbientStrengthLoc, .75f);
            gl.glBindVertexArray(lineVao[CONVEX]);
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, lineEbo[CONVEX]);
            if (selectedExclusiveRender){
                for (Integer i : convexPatchesSelect){
                    SphericalPatch a = convexPatchList.get(i);
                    gl.glDrawElements(GL4.GL_LINES, 2 * a.lineCount, GL4.GL_UNSIGNED_INT, 2 * a.lineOffset * Integer.BYTES);
                }
            } else {
                gl.glDrawElements(GL4.GL_LINES, 2 * convexPatchesEdgeCount, GL4.GL_UNSIGNED_INT, 0);
            }
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
            gl.glBindVertexArray(0);
        }
    }

    private void drawConcave(){
        gl.glUniform3fv(uniNormalColorLoc, 1, concavePatchCol, 0);
        gl.glUniform3fv(uniSelectedColorLoc, 1, selectedPatchCol, 0);

        if (concaveMeshInitialized) {
            if (selectedExclusiveRender){
                if (concavePatchesSelect.size() > 0) {
                    gl.glFrontFace(GL4.GL_CW);
                    gl.glUniform1i(uniSelectedMeshCountLoc, 0);
                    gl.glUniform3fv(uniNormalColorLoc, 1, concavePatchCol, 0);
                    gl.glBindVertexArray(meshVao[CONCAVE]);
                    gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[CONCAVE]);
                    for (Integer i : concavePatchesSelect) {
                        SphericalPatch a = concavePatchList.get(i);
                        int off = (concaveFaceCountShow > a.faces.size()) ? a.faces.size() : concaveFaceCountShow;
                        gl.glDrawElements(GL4.GL_TRIANGLES, 3 * a.faces.size() - 3 * off, GL4.GL_UNSIGNED_INT, a.eboOffset * Integer.BYTES);
                    }
                    gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
                    gl.glBindVertexArray(0);
                    gl.glFrontFace(GL4.GL_CCW);
                }
            } else {
                gl.glUniform1iv(uniSelectedMeshStartLoc, 20, zeroes);
                gl.glUniform1iv(uniSelectedMeshEndLoc, 20, zeroes);
                IntBuffer start = GLBuffers.newDirectIntBuffer(concavePatchesSelect.size());
                IntBuffer end = GLBuffers.newDirectIntBuffer(concavePatchesSelect.size());
                for (Integer i : concavePatchesSelect) {
                    SphericalPatch a = concavePatchList.get(i);
                    start.put(a.vboOffset);
                    end.put(a.vboOffset + a.vertices.size());
                }
                if (concavePatchesSelect.size() == 0) {
                    start = GLBuffers.newDirectIntBuffer(20);
                    for (int i = 0; i < 20; ++i) {
                        start.put(-1);
                    }
                }
                start.rewind();
                end.rewind();
                gl.glFrontFace(GL4.GL_CW);
                gl.glUniform1f(uniAmbientStrengthLoc, 0.5f);
                gl.glUniform1i(uniSelectedMeshCountLoc, concavePatchesSelect.size());
                gl.glUniform1iv(uniSelectedMeshStartLoc, start.capacity(), start);
                gl.glUniform1iv(uniSelectedMeshEndLoc, end.capacity(), end);
                gl.glFrontFace(GL4.GL_CW);
                gl.glUniform3fv(uniMeshColorLoc, 1, concavePatchCol, 0);
                gl.glUniform3fv(uniNormalColorLoc, 1, concavePatchCol, 0);
                gl.glBindVertexArray(meshVao[CONCAVE]);
                gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[CONCAVE]);
                gl.glDrawElements(GL4.GL_TRIANGLES, 3 * concavePatchesFaceCount, GL4.GL_UNSIGNED_INT, 0);
                gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
                gl.glBindVertexArray(0);
                gl.glFrontFace(GL4.GL_CCW);
            }
        }
        if (renderLines) {
            gl.glUniform1f(uniAmbientStrengthLoc, .75f);
            gl.glBindVertexArray(lineVao[CONCAVE]);
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, lineEbo[CONCAVE]);
            if (selectedExclusiveRender){
                for (Integer i : concavePatchesSelect){
                    SphericalPatch cp = concavePatchList.get(i);
                    gl.glDrawElements(GL4.GL_LINES, 2 * cp.lineCount, GL4.GL_UNSIGNED_INT, 2 * cp.lineOffset * Integer.BYTES);
                }
            } else {
                gl.glDrawElements(GL4.GL_LINES, 2 * concavePatchesEdgeCount, GL4.GL_UNSIGNED_INT, 0);
            }
            gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
            gl.glBindVertexArray(0);
        }
    }

    private void drawTori(){
        if (toriMeshInitialized) {
            if (selectedExclusiveRender){
                if (toriPatchesSelect.size() == 0){
                    return;
                }
                gl.glUniform1i(uniSelectedMeshCountLoc, 0);
                gl.glUniform3fv(uniNormalColorLoc, 1, toriPatchCol, 0);
                gl.glBindVertexArray(meshVao[TORUS]);
                for (Integer i : toriPatchesSelect){
                    ToroidalPatch rp = Surface.rectangles.get(i);
                    gl.glDrawArrays(GL4.GL_TRIANGLES, rp.vboOffset, 3 * rp.faces.size());
                }
                gl.glBindVertexArray(0);
                return;
            }
            gl.glUniform1iv(uniSelectedMeshStartLoc, 20, zeroes);
            gl.glUniform1iv(uniSelectedMeshEndLoc, 20, zeroes);
            IntBuffer start = GLBuffers.newDirectIntBuffer(toriPatchesSelect.size());
            IntBuffer end = GLBuffers.newDirectIntBuffer(toriPatchesSelect.size());
            for (Integer i : toriPatchesSelect) {
                ToroidalPatch a = Surface.rectangles.get(i);
                start.put(a.vboOffset);
                end.put(a.vboOffset + (a.vrts.size() / 2));
            }
            if (toriPatchesSelect.size() == 0) {
                start = GLBuffers.newDirectIntBuffer(20);
                for (int i = 0; i < 20; ++i) {
                    start.put(-1);
                }
            }
            start.rewind();
            end.rewind();
            gl.glUniform1f(uniAmbientStrengthLoc, 0.5f);
            gl.glUniform1i(uniSelectedMeshCountLoc, toriPatchesSelect.size());
            gl.glUniform1iv(uniSelectedMeshStartLoc, start.capacity(), start);
            gl.glUniform1iv(uniSelectedMeshEndLoc, end.capacity(), end);
            gl.glUniform3fv(uniMeshColorLoc, 1, toriPatchCol, 0);
            gl.glUniform3fv(uniNormalColorLoc, 1, toriPatchCol, 0);
            //gl.glUniform3fv(uniSelectedColorLoc, 1, selecte)
            gl.glBindVertexArray(meshVao[TORUS]);
            gl.glDrawArrays(GL4.GL_TRIANGLES, 0, 3 * toriPatchesFaceCount);
            gl.glBindVertexArray(0);
        }
    }

    private void meshConcavePatches(int start, int end, boolean waitForOthers){
        stopRendering.set(true);
        List<Point> verts = new ArrayList<>();
        List<Face> faces = new ArrayList<>();
        AdvancingFrontMethod afm = new AdvancingFrontMethod();
        long startTime = System.currentTimeMillis();
        for (int i = start; i < end; ++i) {
            if (!concavePatchesMeshed[i]) {
                SphericalPatch cp = concavePatchList.get(i);
                if (!cp.valid){
                    //MeshRefinement.refinement.enqueue(cp);
                    continue;
                }
                //selectedConcaveP.set(i);
                //List<Point> noveBody = new ArrayList<>();
                //cpFaces = cpAfm.soho(cp.b, Math.toRadians(70), 0.2, 0.3 * (Math.sqrt(3)/2.f), false, noveBody);
                afm.atomComplete = false;
                while (!afm.atomComplete) {
                    faces.clear();
                    verts.clear();
                    afm._initializeConcaveAFM(cp, Math.toRadians(SesConfig.minAlpha), SesConfig.distTolerance, Surface.maxEdgeLen * (Math.sqrt(3) / 2.f));
                    long time = System.currentTimeMillis();
                    afm._mesh2();
                    //meshTime += System.currentTimeMillis() - time;
                    //cpVrts = noveBody;
                    //trianglesCount += cpFaces.size();
                    concavePatchesMeshed[i] = true;
                    if (afm.volpe){
                        System.out.println("concave " + i + " looped");
                    }
                    //if (cpAfm.volpe) {
                     //   looped.add(i);
                    //}
                    //cpUpdate.set(true);
                    //while (cpUpdate.get()) ;
                }
                //MeshRefinement.refinement.enqueue(cp);
            }
        }
        long endTime = System.currentTimeMillis();
        System.out.println("Concave meshed in " + (endTime - startTime) + " ms");
        concaveMeshThreadsCounter.getAndIncrement();
        if (waitForOthers){
            while (concaveMeshThreadsCounter.get() < 4){}
            GLRunnable task = new GLRunnable() {
                @Override
                public boolean run(GLAutoDrawable glAutoDrawable) {
                    pushConcavePatchesToGPU();
                    concaveMeshInitialized = true;
                    return true;
                }
            };
            window.invoke(false, task);
            stopRendering.set(false);
        } else {
            //concavePushData2GPU.set(true);
        }
    }

    private void meshToriPatches(int start, int end, boolean waitForOthers){
        stopRendering.set(true);
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < end; ++i){
            ToroidalPatch tp = Surface.rectangles.get(i);
            PatchUtil.meshToroidalPatch(tp);
        }
        long endTime = System.currentTimeMillis();
        System.out.println("Tori meshed in " + (endTime - startTime) + " ms");
        toriMeshThreadsCounter.getAndIncrement();
        if (waitForOthers){
            while (!toriPushData2GPU.get()){}
            GLRunnable task = new GLRunnable() {
                @Override
                public boolean run(GLAutoDrawable glAutoDrawable) {
                    pushToriMesh2GPU();
                    stopRendering.set(false);
                    return true;
                }
            };
            window.invoke(false, task);

        } else {
            //toriPushData2GPU.set(true);
        }
    }

    private void pushMeshToGPU(List<Point> vrtsNnormals, List<Integer> indices, int bufferObjectsIdx){
        stopRendering.set(true);
        //animator.resume();
        FloatBuffer VNBuffer = GLBuffers.newDirectFloatBuffer(vrtsNnormals.size() * 3);
        for (Point p : vrtsNnormals){
            VNBuffer.put(p.getFloatData());
        }
        IntBuffer indBuffer = GLBuffers.newDirectIntBuffer(indices.size());
        for (Integer i : indices){
            indBuffer.put(i);
        }
        VNBuffer.rewind();
        indBuffer.rewind();
        gl.glBindVertexArray(meshVao[bufferObjectsIdx]);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, meshVbo[bufferObjectsIdx]);
        gl.glBufferData(GL4.GL_ARRAY_BUFFER, VNBuffer.capacity() * Float.BYTES, VNBuffer, GL4.GL_STATIC_DRAW);
        gl.glVertexAttribPointer(0, 3, GL4.GL_FLOAT, false, 6 * Float.BYTES, 0);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(1, 3, GL4.GL_FLOAT, false, 6 * Float.BYTES, 3 * Float.BYTES);
        gl.glEnableVertexAttribArray(1);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, 0);
        gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, meshEbo[bufferObjectsIdx]);
        gl.glBufferData(GL4.GL_ELEMENT_ARRAY_BUFFER, indBuffer.capacity() * Integer.BYTES, indBuffer, GL4.GL_STATIC_DRAW);
        gl.glBindBuffer(GL4.GL_ELEMENT_ARRAY_BUFFER, 0);
        gl.glBindVertexArray(0);
        stopRendering.set(false);
        if (bufferObjectsIdx == CONCAVE){
            concaveMeshInitialized = true;
        } else {
            convexMeshInitialized = true;
        }
    }

    private void pushConvexPatchesToGPU(){
        List<Point> vrtsNnormals = new ArrayList<>();
        List<Integer> indices = new ArrayList<>();
        int vboOffset = 0;
        int eboOffset = 0;
        int faceCount = 0;
        for (SphericalPatch a : Surface.convexPatches){
            for (Point v : a.vertices){
                Point n = new Point(Point.subtractPoints(v, a.sphere.center).makeUnit().getFloatData());
                vrtsNnormals.add(v);
                vrtsNnormals.add(n);
            }
            for (Face f : a.faces){
                indices.add(f.a + vboOffset);
                indices.add(f.b + vboOffset);
                indices.add(f.c + vboOffset);
            }
            a.vboOffset = vboOffset;
            a.eboOffset = eboOffset;
            vboOffset += a.vertices.size();
            eboOffset += 3 * a.faces.size();
            faceCount += a.faces.size();
        }
        convexPatchesFaceCount = indices.size() / 3;
        //System.out.println(convexPatchList.size() + " convex patches");
        pushMeshToGPU(vrtsNnormals, indices, CONVEX);
        convexPushData2GPU.set(false);
        System.out.println("Number of triangles: " + faceCount);
    }

    private void pushConcavePatchesToGPU(){
        List<Point> vrtsNnormals = new ArrayList<>();
        List<Integer> indices = new ArrayList<>();
        int vboOffset = 0;
        int eboOffset = 0;
        int faceCount = 0;
        for (SphericalPatch cp : Surface.triangles){
            for (Point v : cp.vertices){
                Point n = new Point(Point.subtractPoints(cp.sphere.center, v).makeUnit().getFloatData());
                vrtsNnormals.add(v);
                vrtsNnormals.add(n);
            }
            for (Face f : cp.faces){
                indices.add(f.a + vboOffset);
                indices.add(f.b + vboOffset);
                indices.add(f.c + vboOffset);
            }
            cp.vboOffset = vboOffset;
            cp.eboOffset = eboOffset;
            vboOffset += cp.vertices.size();
            eboOffset += 3 * cp.faces.size();
            faceCount += cp.faces.size();
        }
        concavePatchesFaceCount = indices.size() / 3;
        //System.out.println(concavePatchList.size() + " concave patches");
        pushMeshToGPU(vrtsNnormals, indices, CONCAVE);
        concavePushData2GPU.set(false);
        System.out.println("Number of triangles: " + faceCount);
    }

    public void pushConvex(){
        GLRunnable r = new GLRunnable() {
            @Override
            public boolean run(GLAutoDrawable glAutoDrawable) {
                pushConvexPatchesToGPU();
                return true;
            }
        };
        window.invoke(false, r);
    }

    public void pushConcave(){
        GLRunnable r = new GLRunnable() {
            @Override
            public boolean run(GLAutoDrawable glAutoDrawable) {
                pushConcavePatchesToGPU();
                return true;
            }
        };
        window.invoke(false, r);
    }

    private void pushToriMesh2GPU(){
        stopRendering.set(true);
        List<Point> vrtsNormals = new ArrayList<>();
        int vboOffset = 0;
        int faceCount = 0;
        for (ToroidalPatch tp : Surface.rectangles){
            for (Point p : tp.vrts){
                vrtsNormals.add(p);
            }
            tp.vboOffset = vboOffset;
            tp.faceCount = 3 * tp.faces.size();
            vboOffset += tp.vrts.size() / 2;
            faceCount += tp.faces.size();
        }
        FloatBuffer buffer = GLBuffers.newDirectFloatBuffer(vrtsNormals.size() * 3);
        for (Point p : vrtsNormals){
            buffer.put(p.getFloatData());
        }
        buffer.rewind();
        toriPatchesFaceCount = buffer.capacity() / 6;
        gl.glBindVertexArray(meshVao[TORUS]);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, meshVbo[TORUS]);
        gl.glBufferData(GL4.GL_ARRAY_BUFFER, buffer.capacity() * Float.BYTES, buffer, GL4.GL_STATIC_DRAW);
        gl.glVertexAttribPointer(0, 3, GL4.GL_FLOAT, false, 6 * Float.BYTES, 0);
        gl.glEnableVertexAttribArray(0);
        gl.glVertexAttribPointer(1, 3, GL4.GL_FLOAT, false, 6 * Float.BYTES, 3 * Float.BYTES);
        gl.glEnableVertexAttribArray(1);
        gl.glBindBuffer(GL4.GL_ARRAY_BUFFER, 0);
        gl.glBindVertexArray(0);
        stopRendering.set(false);
        toriPushData2GPU.set(false);
        toriMeshInitialized = true;
        System.out.println("Number of triangles: " + faceCount);
    }

    @Override
    public void keyPressed(KeyEvent keyEvent) {

        if (keyEvent.getKeyChar() == ','){
            /*if (!MeshRefinement.refinement.isRunning()){
                MeshRefinement.refinement.start();
            }
            convexPushData2GPU.set(false);
            int half = convexPatchList.size() / 2;
            int step = convexPatchList.size() / threadCount;
            Runnable thread1 = new Runnable() {
                @Override
                public void run() {
                    meshConvexPatches(0, step, false);
                }
            };
            //offset += step;
            Runnable thread2 = new Runnable() {
                @Override
                public void run() {
                    meshConvexPatches(step, 2*step, false);
                }
            };
            Runnable thread3 = new Runnable() {
                @Override
                public void run() {
                    meshConvexPatches(2*step, 3*step, false);
                }
            };
            Runnable thread4 = new Runnable() {
                @Override
                public void run() {
                    meshConvexPatches(3*step, convexPatchList.size(), true);
                }
            };
            (new Thread(thread1)).start();
            (new Thread(thread2)).start();
            (new Thread(thread3)).start();
            (new Thread(thread4)).start();*/
        }

        if (keyEvent.getKeyChar() == '.'){
            if (!MeshRefinement.refinement.isRunning()){
                //MeshRefinement.refinement.start();
            }
            concavePushData2GPU.set(false);
            int half = concavePatchList.size() / 2;
            int step = concavePatchList.size() / 4;
            Runnable task1 = new Runnable() {
                @Override
                public void run() {
                    meshConcavePatches(0, step, false);
                }
            };
            Runnable task2 = new Runnable() {
                @Override
                public void run() {
                    meshConcavePatches(step, 2 * step, false);
                }
            };
            Runnable task3 = new Runnable() {
                @Override
                public void run() {
                    meshConcavePatches(2 * step, 3 * step, false);
                }
            };
            Runnable task4 = new Runnable() {
                @Override
                public void run() {
                    meshConcavePatches(3 * step, concavePatchList.size(), true);
                }
            };
            (new Thread(task1)).start();
            (new Thread(task2)).start();
            (new Thread(task3)).start();
            (new Thread(task4)).start();
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_F1){
            concaveFaceCountShow = (concaveFaceCountShow > 0) ? concaveFaceCountShow - 1 : concaveFaceCountShow;
            convexFaceCountShow = (convexFaceCountShow > 0) ? convexFaceCountShow - 1 : convexFaceCountShow;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_F2){
            concaveFaceCountShow++;
            convexFaceCountShow++;
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_F3){
            concaveFaceCountShow = 0;
            convexFaceCountShow = 0;
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_F5){
            if (concavePatchesSelect.size() > 0) {
                SurfaceParser.exportCP(Surface.triangles.get(concavePatchesSelect.get(0)), "/home/radoslav/objs/cp" + concavePatchesSelect.get(0).toString() + "_" + (int) (Math.random() * 100) + ".obj");
            }
            if (convexPatchesSelect.size() > 0) {
                SurfaceParser.exportCP(Surface.convexPatches.get(convexPatchesSelect.get(0)), "/home/radoslav/objs/cvp" + convexPatchesSelect.get(0).toString() + "_" + (int) (Math.random() * 100) + ".obj");
            }
        }

        if (keyEvent.getKeyChar() == ']'){
            //toriPushData2GPU.set(false);
            int step = Surface.rectangles.size() / 4;
            Runnable t1 = new Runnable() {
                @Override
                public void run() {
                    meshToriPatches(0, Surface.rectangles.size(), true);
                }
            };
            Runnable t2 = new Runnable() {
                @Override
                public void run() {
                    meshToriPatches(step, 2 * step, false);
                }
            };
            Runnable t3 = new Runnable() {
                @Override
                public void run() {
                    meshToriPatches(2 * step, 3 * step, false);
                }
            };
            Runnable t4 = new Runnable() {
                @Override
                public void run() {
                    meshToriPatches(3 * step, Surface.rectangles.size(), true);
                }
            };
            toriPushData2GPU.set(true);
            (new Thread(t1)).start();
            /*(new Thread(t2)).start();
            (new Thread(t3)).start();
            (new Thread(t4)).start();*/
        }

        if (keyEvent.getKeyChar() == 'h'){
            Runnable t1 = new Runnable() {
                @Override
                public void run() {
                    meshConcavePatches(251, 252, true);
                }
            };
            concavePushData2GPU.set(true);
            (new Thread(t1)).start();
        }

        if (keyEvent.getKeyChar() == '\\'){
            stopRendering.set(!stopRendering.get());
        }
        if (keyEvent.getKeyChar() == 'g'){
            /*GLRunnable task = new GLRunnable() {
                @Override
                public boolean run(GLAutoDrawable glAutoDrawable) {
                    sendPatchesLists(Main.convexPatches, Main.triangles);
                    return true;
                }
            };
            window.invoke(false, task);*/
            //convexMeshInitialized = concaveMeshInitialized = toriMeshInitialized;
            convexMeshInitialized = !convexMeshInitialized;
            concaveMeshInitialized = !concaveMeshInitialized;
            toriMeshInitialized = !toriMeshInitialized;
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_F9){
            convexMeshInitialized = !convexMeshInitialized;
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_F10){
            concaveMeshInitialized = !concaveMeshInitialized;
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_F11){
            toriMeshInitialized = !toriMeshInitialized;
        }

        if (Character.isDigit(keyEvent.getKeyChar())){
            strSelectedAtom += keyEvent.getKeyChar();
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_BACK_SPACE){
            strSelectedAtom = "";
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_ENTER){
            try {
                int temp = Integer.parseInt(strSelectedAtom);
                if (temp < convexPatchList.size()){
                    selectedAtom.set(temp);
                } else {
                    strSelectedAtom = "";
                }
            } catch (NumberFormatException e){
                System.out.println("Not a number");
            }
            strSelectedAtom = "";
        }

        if (keyEvent.getKeyChar() == '+'){
            //selectedAtom = (selectedAtom.get() + 1 >= convexPatches.size()) ? 0 : selectedAtom.add(1);
            if (selectedAtom.get() + 1 >= convexPatchList.size()){
                selectedAtom.set(0);
            } else {
                selectedAtom.set(selectedAtom.get() + 1);
            }
            /*if (selectedConcaveP.get() + 1 >= concavePatchList.size()){
                selectedConcaveP.set(0);
            } else {
                selectedConcaveP.set(selectedConcaveP.get() + 1);
            }*/
        }

        if (keyEvent.getKeyChar() == '-'){
            //selectedAtom = (selectedAtom - 1 < 0) ? convexPatches.size() - 1 : selectedAtom - 1;
            if (selectedAtom.get() - 1 < 0){
                selectedAtom.set(convexPatchList.size() - 1);
            } else {
                selectedAtom.set(selectedAtom.get() - 1);
            }
           /* if (selectedConcaveP.get() - 1 < 0){
                selectedConcaveP.set(concavePatchList.size() - 1);
            } else {
                selectedConcaveP.set(selectedConcaveP.get() - 1);
            }*/
        }

        if (keyEvent.getKeyChar() == 'n'){
            camDir = new Quaternion(direction[0], direction[1], direction[2], 0.f);
            camDir.normalize();
            //Atom sel = convexPatches.get(selectedAtom.get());
            //Vector toAtom = Point.subtractPoints(sel.getCenter(), new Point(cameraPos[0], cameraPos[1], cameraPos[2])).makeUnit();
            Vector v = Point.subtractPoints(lastCameraTarget, new Point(cameraPos[0], cameraPos[1], cameraPos[2])).makeUnit();
            camTar = new Quaternion((float)v.getX(), (float)v.getY(), (float)v.getZ(), 0.f);
            camTar.normalize();
            slerping = true;
            slerpParam = 0.0f;
        }

        if (keyEvent.getKeyChar() == 'x'){
            selectedExclusiveRender = !selectedExclusiveRender;
        }

        if (keyEvent.getKeyChar() == 'z'){
            onlyCircular = !onlyCircular;
        }
        if (convexPatchList != null) {
            if (strSelectedAtom.length() > 0) {
                window.setTitle("Selected atom: " + selectedAtom.get() + " / " + convexPatchList.size() + " Atom to select: " + strSelectedAtom + " Press Enter to confirm");
            } else {
                window.setTitle("Selected atom: " + selectedAtom.get() + " / " + convexPatchList.size());
            }
        }

        /*if (keyEvent.getKeyCode() == KeyEvent.VK_SPACE){
            Arc l = Main.convexPatches.get(atomIdx++).arcs.get(0);
            buffersInitialized = false;
            this.storeNewData(l.getVertices(), l.getEdges());
            if (atomIdx >= Main.convexPatches.size()){
                atomIdx = 0;
            }
        }*/
        if (keyEvent.getKeyCode() == KeyEvent.VK_ESCAPE){
            captureMouse = !captureMouse;
            rotating = !captureMouse;
            window.confinePointer(captureMouse);
            window.setPointerVisible(!captureMouse);
            window.warpPointer(window.getWidth() / 2, window.getHeight() / 2);
            dontConsider = true;
        }
        if (keyEvent.getKeyChar() == 'w'){
            /*cameraPos[0] += direction[0] * deltaTime * speed;
            cameraPos[1] += direction[1] * deltaTime * speed;
            cameraPos[2] += direction[2] * deltaTime * speed;*/
            cforward = 1;
            //cameraMoving = true;
        }
        if (keyEvent.getKeyChar() == 's'){
            /*cameraPos[0] -= direction[0] * deltaTime * speed;
            cameraPos[1] -= direction[1] * deltaTime * speed;
            cameraPos[2] -= direction[2] * deltaTime * speed;*/
            cforward = -1;
            //cameraMoving = true;
        }
        if (keyEvent.getKeyChar() == 'a'){
            /*cameraPos[0] -= right[0] * deltaTime * speed;
            cameraPos[1] -= right[1] * deltaTime * speed;
            cameraPos[2] -= right[2] * deltaTime * speed;*/
            cright = -1;
        }
        if (keyEvent.getKeyChar() == 'd'){
            /*cameraPos[0] += right[0] * deltaTime * speed;
            cameraPos[1] += right[1] * deltaTime * speed;
            cameraPos[2] += right[2] * deltaTime * speed;*/
            cright = 1;
        }

        if (keyEvent.getKeyChar() == 'c'){
            lightPos.setX(cameraPos[0]);
            lightPos.setY(cameraPos[1]);
            lightPos.setZ(cameraPos[2]);
        }

        if (keyEvent.getKeyChar() == 'k'){
            cullFaces = !cullFaces;
        }

        if (keyEvent.getKeyCode() == KeyEvent.VK_UP){
            modelY += 0.1f;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_DOWN){
            modelY -= 0.1f;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_LEFT){
            modelX -= 0.1f;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_RIGHT){
            modelX += 0.1f;
        }

        if (keyEvent.getKeyChar() == 'f'){
            drawFaces = !drawFaces;
            drawModeUpdate = true;
        }



        if (keyEvent.getKeyChar() == 'l'){
            renderLines = !renderLines;
            //drawLinesOnTop = !drawLinesOnTop;
        }

        if (keyEvent.getKeyChar() == '/'){
            renderCPs = !renderCPs;
        }




        if (keyEvent.getKeyChar() == 'o'){
            step =  !step;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_SHIFT){
            if (zooming){
                viewPanning = true;
                zooming = false;
            }
            addToSelection = true;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_CONTROL){
            removeSelection = true;
        }
        if (keyEvent.getKeyChar() == 'm'){
            mouseSelect = !mouseSelect;
            if (!mouseSelect){
                hoverAtom = -1;
            }
            /*rayDir[0] = direction[0];
            rayDir[1] = direction[1];
            rayDir[2] = direction[2];*/
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_SPACE){
            List<Neighbor<double[], SphericalPatch>> neighs = new ArrayList<>();
            SphericalPatch cp = concavePatchList.get(selectedConcaveP.get());
            Surface.probeTree.range(cp.sphere.center.getData(), 2 * Double.longBitsToDouble(Surface.probeRadius.get()), neighs);
            System.out.println("found " + neighs.size() + " neighbors");
            linkNeighbors.clear();
            for (Neighbor<double[], SphericalPatch> n : neighs){
                System.out.println("id: " + n.value.id);
                linkNeighbors.add(n.value.id);
            }
        }
    }

    @Override
    public void keyReleased(KeyEvent keyEvent) {
        if (keyEvent.isAutoRepeat()){
            return;
        }
        if (keyEvent.getKeyChar() == 'w' || keyEvent.getKeyChar() == 's'){
            cforward = 0;
        }
        if (keyEvent.getKeyChar() == 'a' || keyEvent.getKeyChar() == 'd'){
            cright = 0;
        }
        if (keyEvent.getKeyCode() == KeyEvent.VK_SHIFT){
            viewPanning = false;
            addToSelection = false;
        }
        if (keyEvent.getKeyCode() == keyEvent.VK_CONTROL){
            removeSelection = false;
        }
    }
    private int hoverSelectID = -1;
    private int getMeshID(){
        if (hoverAtom > -1){
            if (hoverAtom < convexVerticesCount){
                //int idx = 0;
                int localCount = hoverAtom;
                int h = 0;
                for (int i = 0; i < convexPatchList.size(); ++i){
                    SphericalPatch a = convexPatchList.get(i);
                    h += a.vertices.size();
                    if (localCount < h){
                        hoverSelectID = i;
                        break;
                    }
                }
                return CONVEX;
            } else if (hoverAtom < convexVerticesCount + concaveVerticesCount) {
                int localCount = hoverAtom - convexVerticesCount;
                int h = 0;
                //selectedConcaveP.set(hoverAtom - convexPatches.size());
                for (int i = 0; i < concavePatchList.size(); ++i) {
                    h += concavePatchList.get(i).vertices.size();
                    if (localCount < h) {
                        hoverSelectID = i;
                        break;
                    }
                }
                return CONCAVE;
            } else {
                int localCount = hoverAtom - convexVerticesCount - concaveVerticesCount;
                System.out.println("loc count: " + localCount);
                int h = 0;
                //selectedToriP.set(hoverAtom - convexPatches.size() - concavePatchList.size());
                for (int i = 0; i < Surface.rectangles.size(); ++i){
                    h += Surface.rectangles.get(i).vrts.size() / 2;
                    if (localCount < h){
                        hoverSelectID = i;
                        break;
                    }
                }
                return TORUS;
            }
        }
        return -1;
    }

    @Override
    public void mouseClicked(MouseEvent mouseEvent) {
        if (hoverAtom > -1){
            int meshType = getMeshID();
            if (meshType == CONVEX){
                selectedAtom.set(hoverSelectID);
                if (!addToSelection && !removeSelection){
                    convexPatchesSelect.clear();
                    convexPatchesSelect.add(hoverSelectID);
                } else {
                    if (addToSelection) {
                        convexPatchesSelect.add(hoverSelectID);
                    } else if (removeSelection) {
                        convexPatchesSelect.remove((Object)hoverSelectID);
                    }
                }
                System.out.println("atom id: " + hoverSelectID);
            } else if (meshType == CONCAVE){
                selectedConcaveP.set(hoverSelectID);
                if (!addToSelection && !removeSelection){
                    concavePatchesSelect.clear();
                    concavePatchesSelect.add(hoverSelectID);
                } else {
                    if (addToSelection){
                        concavePatchesSelect.add(hoverSelectID);
                    } else if (removeSelection){
                        concavePatchesSelect.remove((Object)hoverSelectID);
                    }
                }
                System.out.println("triangle id: " + hoverSelectID);
                linkNeighbors.clear();
            } else {
                selectedToriP.set(hoverSelectID);
                if (!addToSelection && !removeSelection){
                    toriPatchesSelect.clear();
                    toriPatchesSelect.add(hoverSelectID);

                } else {
                    if (addToSelection){
                        toriPatchesSelect.add(hoverSelectID);
                    } else if (removeSelection){
                        toriPatchesSelect.remove((Object)hoverSelectID);
                    }
                }
                System.out.println("tori id: " + hoverSelectID);
            }
        }
        /*if (hoverAtom > -1){
            if (hoverAtom < convexVerticesCount) {
                //selectedAtom.set(hoverAtom);
                int idx = 0;
                int localCount = hoverAtom;
                int h = 0;
                for (int i = 0; i < convexPatches.size(); ++i){
                    Atom a = convexPatches.get(i);
                    h += a.vertices.size();
                    if (localCount < h){
                        idx = i;
                        break;
                    }
                }
                selectedAtom.set(idx);
                if (!addToSelection && !removeSelection){
                    convexPatchesSelect.clear();
                    convexPatchesSelect.add(idx);
                } else {
                    if (addToSelection) {
                        convexPatchesSelect.add(idx);
                    } else if (removeSelection) {
                        convexPatchesSelect.remove((Object)idx);
                    }
                }
                System.out.println("atom id: " + idx);

            } else if (hoverAtom < convexVerticesCount + concaveVerticesCount){
                int localCount = hoverAtom - convexVerticesCount;
                int idx = 0;
                int h = 0;
                //selectedConcaveP.set(hoverAtom - convexPatches.size());
                for (int i = 0; i < concavePatchList.size(); ++i){
                    h += concavePatchList.get(i).vertices.size();
                    if (localCount < h){
                        idx = i;
                        break;
                    }
                }
                if (!addToSelection && !removeSelection){
                    concavePatchesSelect.clear();
                    concavePatchesSelect.add(idx);
                } else {
                    if (addToSelection){
                        concavePatchesSelect.add(idx);
                    } else if (removeSelection){
                        concavePatchesSelect.remove((Object)idx);
                    }
                }
                System.out.println("triangle id: " + idx);
                //
                linkNeighbors.clear();
            } else {
                int localCount = hoverAtom - convexVerticesCount - concaveVerticesCount;
                int idx = 0;
                int h = 0;
                //selectedToriP.set(hoverAtom - convexPatches.size() - concavePatchList.size());
                for (int i = 0; i < Main.rectangles.size(); ++i){
                    h += Main.rectangles.get(i).vertices.size();
                    if (localCount < h){
                        idx = i;
                        break;
                    }
                }
                if (!addToSelection && !removeSelection){
                    toriPatchesSelect.clear();
                    toriPatchesSelect.add(idx);

                } else {
                    if (addToSelection){
                        toriPatchesSelect.add(idx);
                    } else if (removeSelection){
                        toriPatchesSelect.remove((Object)idx);
                    }
                }
                System.out.println("tori id: " + idx);
            }
        }*/
    }

    @Override
    public void mouseEntered(MouseEvent mouseEvent) {

    }

    @Override
    public void mouseExited(MouseEvent mouseEvent) {

    }

    @Override
    public void mousePressed(MouseEvent mouseEvent) {
        if (!mouseDown){
            mouseDown = true;
            //arcStart = new Vector(mouseEvent.getX() / (double)window.getWidth(), mouseEvent.getY() / (double)window.getHeight(), 0);

        }
        //System.out.println("mouse butt: " + mouseEvent.getButton());
        if (mouseEvent.getButton() == MouseEvent.BUTTON2){
            if (mouseEvent.isShiftDown()){
                viewPanning = true;
            } else {
                zooming = true;
            }
            zoomSpeed = (mouseEvent.isControlDown()) ? 0.01f : 0.1f;
            window.setPointerVisible(false);
            window.confinePointer(true);
            window.warpPointer(window.getWidth() / 2, window.getHeight() / 2);
            rotating = false;
        }
        if (rotating){
            lastX = mouseEvent.getX();
            lastY = mouseEvent.getY();
        }
    }

    @Override
    public void mouseReleased(MouseEvent mouseEvent) {
        mouseDown = false;
        if (zooming) {
            if (!captureMouse) {
                window.setPointerVisible(true);
                window.confinePointer(false);
            }
            zooming = false;
        }
        if (viewPanning){
            if (!captureMouse){
                window.setPointerVisible(true);
                window.confinePointer(false);
            }
            viewPanning = false;
        }
        rotating = true;
    }

    @Override
    public void mouseMoved(MouseEvent mouseEvent) {
        lastX = mouseEvent.getX();
        lastY = mouseEvent.getY();
        if (captureMouse) {
            mouseAngleX = mouseSpeed * deltaTime * (window.getWidth() / 2.f - mouseEvent.getX());
            mouseAngleY = mouseSpeed * deltaTime * (window.getHeight() / 2.f - mouseEvent.getY());

            if (mouseAngleY > 90.f){
                mouseAngleY = 89.f;
            } else if (mouseAngleY < -90.f){
                mouseAngleY = -89.f;
            }
            Quaternion axisUp = new Quaternion(0.f, 0.f, 0.f, 0.f);
            axisUp.setFromAngleNormalAxis((float)Math.toRadians(mouseAngleX) * 1.f, up);
            direction = axisUp.rotateVector(direction, 0, direction, 0);
            direction = VectorUtil.normalizeVec3(direction);
            /*if (Math.abs(VectorUtil.dotVec3(direction, new float[]{0.f, 1.f, 0.f}) - 1.f) < 0.001){
                right = VectorUtil.crossVec3(right, new float[]{0.f, 0.f, 1.f}, direction);
                System.out.println("BINGO");
                //up = VectorUtil.crossVec3(up, right, direction);
                if (mouseAngleY < 0.f){
                    mouseAngleY = 0;
                }
            } else {
                right = VectorUtil.crossVec3(right, new float[]{0.f, 1.f, 0.f}, direction);
            }*/
            /*if (Math.abs(VectorUtil.dotVec3(direction, new float[]{0.f, 1.f, 0.f}) - Math.cos(Math.toRadians(87))) < 0.001 && mouseAngleY < 0){
                return;
            }*/
            window.warpPointer(window.getWidth() / 2, window.getHeight() / 2);
            if (VectorUtil.dotVec3(direction, new float[]{0.f, 1.f, 0.f}) > Math.cos(Math.toRadians(10)) && mouseAngleY < 0){
                right = VectorUtil.crossVec3(right, new float[]{0.f, 1.f, 0.f}, direction);
                return;
            }
            if (VectorUtil.dotVec3(direction, new float[]{0.f, -1.f, 0.f}) > Math.cos(Math.toRadians(-10)) && mouseAngleY > 0){
                right = VectorUtil.crossVec3(right, new float[]{0.f, 1.f, 0.f}, direction);
                return;
            }
            right = VectorUtil.crossVec3(right, new float[]{0.f, 1.f, 0.f}, direction);

            right = VectorUtil.normalizeVec3(right);
            up = VectorUtil.crossVec3(up, right, direction);
            Quaternion axisRight = new Quaternion(0.f, 0.f, 0.f, 0.f);
            axisRight.setFromAngleNormalAxis((float)Math.toRadians(mouseAngleY) * 1.f, right);
            direction = axisRight.rotateVector(direction, 0, direction, 0);
            up = VectorUtil.crossVec3(up, right, direction);
            up = VectorUtil.normalizeVec3(up);

        }
        if (mouseSelect){
            mouseLocation.x = mouseEvent.getX();
            mouseLocation.y = mouseEvent.getY();
            moved = true;
        }
    }

    @Override
    public void mouseDragged(MouseEvent mouseEvent) {
        if (zooming){
            if (mouseEvent.isControlDown()){
                zoomSpeed = 0.01f;
            } else {
                zoomSpeed = 0.1f;
            }
            int diffy = (window.getHeight() / 2 - mouseEvent.getY());
            cameraPos[0] += zoomSpeed * direction[0] * diffy;
            cameraPos[1] += zoomSpeed * direction[1] * diffy;
            cameraPos[2] += zoomSpeed * direction[2] * diffy;
            window.warpPointer(window.getWidth() / 2, window.getHeight() / 2);
        }
        if (viewPanning){
            if (mouseEvent.isControlDown()){
                zoomSpeed = 0.01f;
            } else {
                zoomSpeed = 0.1f;
            }
            int diffx = (window.getWidth() / 2 - mouseEvent.getX());
            int diffy = (window.getHeight() / 2 - mouseEvent.getY());
            cameraPos[0] -= zoomSpeed * (-right[0] * diffx + up[0] * diffy);
            cameraPos[1] -= zoomSpeed * (-right[1] * diffx + up[1] * diffy);
            cameraPos[2] -= zoomSpeed * (-right[2] * diffx + up[2] * diffy);
            window.warpPointer(window.getWidth() / 2, window.getHeight() / 2);
        }
        if (rotating){
            int diffx = mouseEvent.getX() - lastX;
            int diffy = mouseEvent.getY() - lastY;
            angleX += 0.01 * diffx;
            angleY += 0.01 * diffy;
            modelMAT.glLoadIdentity();
            modelMAT.glTranslatef((float) Surface.centerOfgravity.x, (float) Surface.centerOfgravity.y, (float) Surface.centerOfgravity.z);
            Quaternion q1 = new Quaternion(0, 0, 0, 0);
            q1.setFromAngleNormalAxis((float)angleX, up);
            Quaternion q2 = new Quaternion(0, 0, 0, 0);
            q2.setFromAngleNormalAxis((float)-angleY, right);
            modelMAT.glRotate(q1);
            modelMAT.glRotate(q2);
            modelMAT.glTranslatef((float)-Surface.centerOfgravity.x, (float)-Surface.centerOfgravity.y, (float)-Surface.centerOfgravity.z);
            lastX = mouseEvent.getX();
            lastY = mouseEvent.getY();
        }
    }

    @Override
    public void mouseWheelMoved(MouseEvent mouseEvent) {
        if (mouseEvent.getRotation()[1] < 0.0f){
            scaleFactor -= 0.5f;
        } else if (mouseEvent.getRotation()[1] > 0.0f){
            scaleFactor += 0.5f;
        }
        selectedMeshScaleMat.loadIdentity();
        selectedMeshScaleMat.scale(scaleFactor, scaleFactor, scaleFactor);
    }

    public void freeGLResources(){
        /*animator.stop();
        while (animator.isAnimating()){}
        render = false;*/

        //while (!stoppedRendering.get());
        /*GLRunnable task = new GLRunnable() {
            @Override
            public boolean run(GLAutoDrawable glAutoDrawable) {*/
        System.out.println("DELETING THE WHOLE THING");
                IntBuffer buff = GLBuffers.newDirectIntBuffer(vaos.size());
                stopRendering.set(true);
                while (!stoppedRendering.get()){
                    System.out.println("waiting for stop");
                }
                /*if (convexPatches != null && convexPatches.size() > 0){
                    convexPatches = null;
                }
                if (concavePatchList != null && concavePatchList.size() > 0){
                    concavePatchList.clear();
                }*/
                convexPatchList = null;
                concavePatchList = null;
                if (vaos.size() > 0) {
                    for (Integer i : vaos) {
                        buff.put(i);
                    }
                    buff.rewind();
                    gl.glDeleteVertexArrays(vaos.size(), buff);
                }

                if (vbos.size() > 0) {
                    buff = GLBuffers.newDirectIntBuffer(vbos.size());
                    for (Integer i : vbos) {
                        buff.put(i);
                    }
                    buff.rewind();
                    gl.glDeleteBuffers(buff.capacity(), buff);
                }
                if (ebos.size() > 0){
                    buff = GLBuffers.newDirectIntBuffer(ebos.size());
                    for (Integer i : ebos) {
                        buff.put(i);
                    }
                    buff.rewind();
                    gl.glDeleteBuffers(buff.capacity(), buff);
                }
                vaos.clear();
                vbos.clear();
                ebos.clear();
                gl.glDeleteVertexArrays(3, meshVao, 0);
                gl.glDeleteBuffers(3, meshVbo, 0);
                gl.glDeleteBuffers(2, meshEbo, 0);

                gl.glDeleteVertexArrays(2, lineVao, 0);
                gl.glDeleteBuffers(2, lineVbo, 0);
                gl.glDeleteBuffers(2, lineEbo, 0);

                gl.glGenVertexArrays(3, meshVao, 0);
                gl.glGenBuffers(3, meshVbo, 0);
                gl.glGenBuffers(2, meshEbo, 0);

                gl.glGenVertexArrays(2, lineVao, 0);
                gl.glGenBuffers(2, lineVbo, 0);
                gl.glGenBuffers(2, lineEbo, 0);
                //return true;
        resourcesFreed.set(true);
        System.out.println("resources freed");
            //}
        //};
        //window.invoke(false, task);
        //convexPatches.clear();
        //concavePatchList.clear();
        //Main.rectangles.clear();

    }

    public void close(){
        /*GLRunnable task = new GLRunnable() {
            @Override
            public boolean run(GLAutoDrawable glAutoDrawable) {
                freeGLResources();
                return true;
            }
        };
        window.invoke(true, task);
        animator.stop();
        window.destroy();*/
        window.destroy();
    }

    public void setProbeAlpha(float v){
        this.probeAlpha = v;
    }

    public void setMouseSensitivity(float f){
        this.mouseSpeed = f;
    }

    public void stopRendering(boolean v){
        stopRendering.set(v);
    }

    public void requestFreeResources(){
        GLRunnable task = new GLRunnable() {
            @Override
            public boolean run(GLAutoDrawable glAutoDrawable) {
                freeGLResources();
                return true;
            }
        };
        window.invoke(false, task);
    }

    public boolean getResourcesFreed(){
        return resourcesFreed.get();
    }
}
