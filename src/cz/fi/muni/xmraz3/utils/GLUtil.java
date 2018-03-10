package cz.fi.muni.xmraz3.utils;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL4;
import com.jogamp.opengl.GLContext;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.GLBuffers;
import cz.fi.muni.xmraz3.Face;
import cz.fi.muni.xmraz3.math.Point;
import graphicslib3D.Matrix3D;

import java.io.*;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import static com.jogamp.opengl.GL2ES2.*;

public class GLUtil {

    public static int createShaderProgram(String vert, String frag){
        GL4 gl = (GL4) GLContext.getCurrentGL();

        /*String vshaders[] =
                {
                        "#version 430 \n",
                        "void main(void) \n",
                        "{ gl_Position = vec4(0.0, 0.0, 0.0, 1.0); } \n"
                };
        String fshaders[] =
                {
                        "#version 430 \n",
                        "out vec4 color; \n",
                        "void main(void) \n",
                        "{ if (gl_FragCoord.x > 512) color = vec4(1.0, 0.0, 1.0, 1.0); else color = vec4(0.0, 1.0, 0.0, 1.0);}"
                };*/
        String[] vshaders = readShaderSource(vert);
        String[] fshaders = readShaderSource(frag);

        int[] vertCompiled = new int[1];
        int[] fragCompiled = new int[1];
        int[] linked = new int[1];
        int vshader = gl.glCreateShader(GL_VERTEX_SHADER);
        gl.glShaderSource(vshader, vshaders.length, vshaders, null, 0);
        gl.glCompileShader(vshader);

        checkOpenGLError();
        gl.glGetShaderiv(vshader, GL_COMPILE_STATUS, vertCompiled, 0);

        if (vertCompiled[0] == 1)
        { System.out.println(". . . vertex compilation success.");
        } else
        { System.out.println(". . . vertex compilation failed.");
            printShaderLog(vshader);
        }

        int fshader = gl.glCreateShader(GL_FRAGMENT_SHADER);
        gl.glShaderSource(fshader, fshaders.length, fshaders, null, 0);
        gl.glCompileShader(fshader);

        checkOpenGLError();
        gl.glGetShaderiv(fshader, GL_COMPILE_STATUS, fragCompiled, 0);

        if (fragCompiled[0] == 1)
        { System.out.println(". . . fragment compilation success.");
        } else
        { System.out.println(". . . fragment compilation failed.");
            printShaderLog(fshader);
        }

        if ((vertCompiled[0] != 1) || (fragCompiled[0] != 1))
        { System.out.println("\nCompilation error; return-flags:");
            System.out.println(" vertCompiled = " + vertCompiled[0] + " ; fragCompiled = " + fragCompiled[0]);
        } else
        {
            System.out.println("Successful compilation");
        }

        int vfprogram = gl.glCreateProgram();
        gl.glAttachShader(vfprogram, vshader);
        gl.glAttachShader(vfprogram, fshader);
        gl.glLinkProgram(vfprogram);
        checkOpenGLError();
        gl.glGetProgramiv(vfprogram, GL_LINK_STATUS, linked,0);
        if (linked[0] == 1) {
            System.out.println(". . . linking succeeded.");
        } else {
            System.out.println(". . . linking failed.");
            printProgramLog(vfprogram);
        }
        gl.glDeleteShader(vshader);
        gl.glDeleteShader(fshader);
        return vfprogram;
    }

    static boolean checkOpenGLError(){
        GL4 gl = (GL4) GLContext.getCurrentGL();
        boolean foundErr = false;
        GLU glu = new GLU();
        int glErr = gl.glGetError();
        while (glErr != GL_NO_ERROR){
            System.err.println("glError: " + glu.gluErrorString(glErr));
            foundErr = true;
            glErr = gl.glGetError();
        }
        return foundErr;
    }

    private static void printProgramLog(int prog){
        GL4 gl = (GL4) GLContext.getCurrentGL();
        int[] len = new int[1];
        int[] chWritten = new int[1];
        byte[] log = null;

        gl.glGetProgramiv(prog, GL_INFO_LOG_LENGTH, len, 0);
        if (len[0] > 0){
            log = new byte[len[0]];
            gl.glGetProgramInfoLog(prog, len[0], chWritten, 0, log, 0);
            System.out.println("Program info log: ");
            for (int i = 0; i < chWritten[0]; ++i){
                System.out.print((char)log[i]);
            }
        }
    }

    private static void printShaderLog(int shader){
        GL4 gl = (GL4) GLContext.getCurrentGL();
        int[] len = new int[1];
        int[] chWritten = new int[1];
        byte[] log = null;
        gl.glGetShaderiv(shader, GL_INFO_LOG_LENGTH, len, 0);
        if (len[0] > 0){
            log = new byte[len[0]];
            gl.glGetShaderInfoLog(shader, len[0], chWritten, 0, log, 0);
            System.out.println("Shader info log: ");
            for (int i = 0; i < chWritten[0]; ++i){
                System.out.print((char)log[i]);
            }
        }
    }

    private static String[] readShaderSource(String file){
        List<String> lines;
        try(BufferedReader br = new BufferedReader(new FileReader(file))){
            lines = br.lines().map(e -> e.concat("\n")).collect(Collectors.toList());
        } catch (IOException e){
            e.printStackTrace();
            System.exit(-1);
            return null;
        }
        /*for (String l : lines){
            System.out.println(l);
        }*/
        return lines.toArray(new String[1]);
    }

    public static Matrix3D perspective(float fovy, float aspect, float n, float f){
        float q = 1.0f / ((float) Math.tan(Math.toRadians(0.5f * fovy)));
        float A = q / aspect;
        float B = (n + f) / (n - f);
        float C = (2.f * n * f) / (n - f);
        Matrix3D r = new Matrix3D();
        r.setElementAt(0, 0, A);
        r.setElementAt(1, 1, q);
        r.setElementAt(2, 2, B);
        r.setElementAt(3, 2, -1.0f);
        r.setElementAt(2, 3, C);
        r.setElementAt(3, 3, 0.f);
        return r;
    }

    public static int[] loadSphere(String file, GL4 gl){
        int[] glObjects = new int[3];
        glObjects[0] = -1;
        glObjects[1] = -1;
        glObjects[2] = 0; //this one indicates number of vertices - it will be used in glDrawArrays calls
        try(BufferedReader br = new BufferedReader(new FileReader(file))){
            List<Point> vrts = new ArrayList<>();
            List<Point> normals = new ArrayList<>();
            List<Face> faceVrts = new ArrayList<>();
            List<Face> faceNormals = new ArrayList<>();
            String line;
            while ((line = br.readLine()) != null){
                if (line.charAt(0) == 'v'){
                    String[] splitt = line.split(" ");
                    float v1 = Float.parseFloat(splitt[1]);
                    float v2 = Float.parseFloat(splitt[2]);
                    float v3 = Float.parseFloat(splitt[3]);
                    Point p = new Point(v1, v2, v3);
                    if (line.charAt(1) == ' '){
                        vrts.add(p);
                    } else {
                        normals.add(p);
                    }
                } else if (line.charAt(0) == 'f'){
                    String[] splitt = line.split(" ");
                    int[] vertexIndices = new int[3];
                    int[] normalIndices = new int[3];
                    for (int i = 1; i < splitt.length; ++i){
                        String[] indices = splitt[i].split("//");
                        vertexIndices[i - 1] = Integer.parseInt(indices[0]);
                        normalIndices[i - 1] = Integer.parseInt(indices[1]);
                    }
                    faceVrts.add(new Face(vertexIndices[0], vertexIndices[1], vertexIndices[2]));
                    faceNormals.add(new Face(normalIndices[0], normalIndices[1], normalIndices[2]));
                }
            }
            FloatBuffer vbuffer = GLBuffers.newDirectFloatBuffer(3 * 3 * 2 * faceNormals.size()); //allocate space for all faces consisting of vertices and their normals
            for (int i = 0; i < faceVrts.size(); ++i){
                Face fV = faceVrts.get(i);
                Face fN = faceNormals.get(i);
                vbuffer.put(vrts.get(fV.a - 1).getFloatData());
                vbuffer.put(normals.get(fN.a - 1).getFloatData());
                vbuffer.put(vrts.get(fV.b - 1).getFloatData());
                vbuffer.put(normals.get(fN.b - 1).getFloatData());
                vbuffer.put(vrts.get(fV.c - 1).getFloatData());
                vbuffer.put(normals.get(fN.c - 1).getFloatData());
            }
            vbuffer.rewind();
            glObjects[2] = faceNormals.size();
            gl.glGenVertexArrays(1, glObjects, 0);
            gl.glGenBuffers(1, glObjects, 1);
            gl.glBindVertexArray(glObjects[0]);
            gl.glBindBuffer(GL.GL_ARRAY_BUFFER, glObjects[1]);
            gl.glBufferData(GL.GL_ARRAY_BUFFER, vbuffer.capacity() * Float.BYTES, vbuffer, GL.GL_STATIC_DRAW);
            gl.glVertexAttribPointer(0, 3, GL_FLOAT, false, 6 * Float.BYTES, 0);
            gl.glVertexAttribPointer(1, 3, GL_FLOAT, false, 6 * Float.BYTES, 3 * Float.BYTES);
            gl.glEnableVertexAttribArray(0);
            gl.glEnableVertexAttribArray(1);
            gl.glBindBuffer(GL.GL_ARRAY_BUFFER, 0);
            gl.glBindVertexArray(0);
        } catch (IOException e){
            e.printStackTrace();
        }
        return glObjects;
    }
}
