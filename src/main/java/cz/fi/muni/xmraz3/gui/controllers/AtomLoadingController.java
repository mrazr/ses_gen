package cz.fi.muni.xmraz3.gui.controllers;

import cz.fi.muni.xmraz3.Surface;
import cz.fi.muni.xmraz3.SurfaceParser;
import cz.fi.muni.xmraz3.gui.MainPanel;
import cz.fi.muni.xmraz3.gui.MainWindow;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.concurrent.Task;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.stage.Stage;

public class AtomLoadingController {
    public static Stage stage;
    public static String folder;
    @FXML
    private Button btnCancel;
    @FXML
    private ProgressBar prgLoading;
    @FXML
    private Label lblProgress;
    public static Task<Void> task;


    @FXML
    public void initialize() {
        btnCancel.setText("Cancel");
        btnCancel.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                task.cancel();
                stage.close();
            }
        });
        lblProgress.setText("Progress:");
        task = new Task<Void>() {
            @Override
            protected Void call() throws Exception {
                Surface.atomsProcessed.addListener(new ChangeListener<Number>() {
                    @Override
                    public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                        updateProgress(newValue.longValue(), Surface.convexPatches.size());
                        updateMessage("Progress: " + newValue.longValue() + " / " + Surface.convexPatches.size());
                    }
                });
                if (MainPanel.atomView == null){
                    MainPanel.atomView = new MainWindow();
                    MainPanel.atomView.setup();
                    MainPanel.atomView.controlPanel = MainPanelController.root;
                }
                MainPanel.atomView.stopRendering(true);
                MainPanel.atomView.requestFreeResources();
                while(!MainPanel.atomView.getResourcesFreed());
                SurfaceParser.ses_start(folder);
                return null;
            }
        };
        prgLoading.progressProperty().bind(task.progressProperty());
        lblProgress.textProperty().bind(task.messageProperty());
        lblProgress.widthProperty().addListener(new ChangeListener<Number>() {
            @Override
            public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                lblProgress.setLayoutX(stage.getWidth() / 2.0 - (lblProgress.getWidth() / 2.0));
            }
        });
    }
    public static void work(){
        new Thread(task).start();
    }
}
