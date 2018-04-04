package cz.fi.muni.xmraz3;

import com.beust.jcommander.IParameterValidator;
import com.beust.jcommander.ParameterException;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class ValidatePath implements IParameterValidator {
    public void validate(String name, String value) throws ParameterException {
        if (name.compareTo("-obj") == 0 || name.compareTo("-stl") == 0) {
            Path p = Paths.get(value);
            if (Files.isDirectory(p)){
                return;
            }
            if (p.getParent() != null && !Files.isDirectory(p.getParent())){
                throw new ParameterException("Invalid export directory: " + p.getParent().toString());
            }
        } else {
            Path p = Paths.get(value);
            if (!Files.isDirectory(p)){
                throw new ParameterException("Invalid input directory: " + value);
            }
        }
    }
}
