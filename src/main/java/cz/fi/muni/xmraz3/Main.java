package cz.fi.muni.xmraz3;

import com.beust.jcommander.JCommander;
import com.beust.jcommander.Parameter;
import com.beust.jcommander.ParameterException;
import cz.fi.muni.xmraz3.gui.MainPanel;

import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class Main {
    public static void main(String[] args) {
        JCommander jc = JCommander.newBuilder().addObject(SesConfig.sesconfig).build();
        jc.parse(args);
        if (SesConfig.help){
            jc.usage();
            return;
        }
        if (SesConfig.inputFolder == null && (SesConfig.objFile != null || SesConfig.stlFile != null)){
            throw new ParameterException("Missing input folder argument");
        }
        if (SesConfig.objFile != null){
            Path p = Paths.get(SesConfig.objFile);
            if (Files.isDirectory(p)){
                SesConfig.objFile = p.resolve(Paths.get(SesConfig.inputFolder).getFileName().toString() + ".obj").toString();
            }
        }
        if (SesConfig.stlFile != null){
            Path p = Paths.get(SesConfig.stlFile);
            if (Files.isDirectory(p)){
                SesConfig.stlFile = p.resolve(Paths.get(SesConfig.inputFolder).getFileName().toString() + ".stl").toString();
            }
        }
        if (SesConfig.useGUI){
            MainPanel.startGUI(args);
        } else {
            if (SesConfig.inputFolder == null){
                jc.usage();
                return;
            }
            //try {
            //    System.in.read();
            //} catch (Exception e){
            //    e.printStackTrace();
            //}
            String raw = SurfaceParser.loadFile(Paths.get(SesConfig.inputFolder).resolve("info.json").toString());
            SurfaceParser.parseSesConfig(raw);
            SurfaceParser.ses_start(SesConfig.inputFolder);
        }
    }
}
