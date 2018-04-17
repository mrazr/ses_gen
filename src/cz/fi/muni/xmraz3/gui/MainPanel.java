package cz.fi.muni.xmraz3.gui;

import cz.fi.muni.xmraz3.SurfaceParser;
import cz.fi.muni.xmraz3.gui.controllers.AtomLoadingController;
import cz.fi.muni.xmraz3.gui.controllers.MainPanelController;
import javafx.application.Application;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Rectangle2D;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

import java.io.IOException;

public class MainPanel extends Application {

    public static MainWindow atomView;
    public static boolean pinnedToView = false;
    public static MainPanelController controlPanel;
    @Override
    public void start(Stage primaryStage) {
        MainPanelController.root = primaryStage;
        FXMLLoader loader = new FXMLLoader();
        loader.setLocation(MainPanel.class.getResource("layout/MainPanelView.fxml"));
        AnchorPane pane = null;
        try {
            pane = (AnchorPane) loader.load();
            Scene scene = new Scene(pane);
            primaryStage.setScene(scene);

            primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>() {
                @Override
                public void handle(WindowEvent event) {
                    if (atomView != null){
                        atomView.close();
                        atomView = null;
                    }
                    if (AtomLoadingController.task != null && AtomLoadingController.task.isRunning()){
                        AtomLoadingController.task.cancel();
                    }
                    //System.out.println("BEFORE EXIT");
                    //SurfaceParser.getMemory();
                    System.exit(0);
                }
            });
            //primaryStage.setResizable(false); //causes different dimensions than from those when resizable is set to true
            primaryStage.show();
            primaryStage.xProperty().addListener(new ChangeListener<Number>() {
                @Override
                public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                    if (atomView != null && pinnedToView && primaryStage.isFocused()){
                        unpinView();
                    }
                }
            });

            primaryStage.yProperty().addListener(new ChangeListener<Number>() {
                @Override
                public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
                    if (atomView != null && pinnedToView && primaryStage.isFocused()){
                        unpinView();
                    }
                }
            });
            /*Rectangle2D primScreenBounds = Screen.getPrimary().getVisualBounds();
            primaryStage.setX((primScreenBounds.getWidth() - primaryStage.getWidth()) / 2);
            primaryStage.setY((primScreenBounds.getHeight() - primaryStage.getHeight()) / 2);*/

        } catch (IOException e){
            e.printStackTrace();
        }
        primaryStage.show();
        /*atomView = new MainWindow();
        atomView.setup();*/
    }

    private void unpinView(){
        pinnedToView = false;
        controlPanel.setCheckPinned(false);
        MainPanel.atomView.isPinned.setValue(false);
    }
    public static void startGUI(String[] args){
        launch(args);
    }
}
