import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QMessageBox
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk

class GUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # Main window settings
        self.setWindowTitle("Branching Vine Robots")
        self.setGeometry(100, 100, 800, 600)

        # Create tab widget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Add 3D and 2D tabs
        self.add_3d_tab()
        self.add_2d_tab()

    def add_3d_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Create VTK Widget
        vtk_widget = QVTKRenderWindowInteractor(tab)
        layout.addWidget(vtk_widget)
        tab.setLayout(layout)

        # Renderer
        renderer = vtk.vtkRenderer()
        vtk_widget.GetRenderWindow().AddRenderer(renderer)
        interactor = vtk_widget.GetRenderWindow().GetInteractor()
        interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        interactor.SetInteractorStyle(interactor_style)
        
        # Mouse click logic
        def on_left_button_press(obj, event):
            click_pos = interactor.GetEventPosition()
            print(f"Mouse clicked at: {click_pos}")
        interactor.AddObserver("LeftButtonPressEvent", on_left_button_press)

        # Example 3D object (sphere)
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(1.0)
        sphere.SetThetaResolution(50)
        sphere.SetPhiResolution(50)
        sphere.Update()  # Ensure the data is generated

        # Create a scalar array and assign it to the sphere
        scalars = vtk.vtkFloatArray()
        scalars.SetNumberOfTuples(sphere.GetOutput().GetNumberOfPoints())

        # Set scalar values based on the radius (or any other attribute)
        for i in range(sphere.GetOutput().GetNumberOfPoints()):
            scalars.SetValue(i, sphere.GetOutput().GetPoints().GetPoint(i)[2])  # Use the Z-coordinate for gradient

        sphere.GetOutput().GetPointData().SetScalars(scalars)

        # Create a color transfer function (gradient)
        color_transfer_function = vtk.vtkColorTransferFunction()
        color_transfer_function.AddRGBPoint(-1.0, 1.0, 0.0, 0.0)  # Red
        color_transfer_function.AddRGBPoint(0.0, 0.0, 1.0, 0.0)   # Green
        color_transfer_function.AddRGBPoint(1.0, 0.0, 0.0, 1.0)   # Blue

        # Create a polydata mapper for the sphere
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        # Set the scalar range and color transfer function to the mapper
        mapper.SetScalarRange(-1.0, 1.0)
        mapper.SetColorModeToMapScalars()
        mapper.SetLookupTable(color_transfer_function)

        # Create an actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Add actor and render
        renderer.AddActor(actor)
        renderer.ResetCamera()
        interactor.Initialize()

        # Add tab
        self.tabs.addTab(tab, "3D View")

    def add_2d_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Create VTK Widget for 2D
        vtk_widget = QVTKRenderWindowInteractor(tab)
        layout.addWidget(vtk_widget)
        tab.setLayout(layout)

        # Renderer
        renderer = vtk.vtkRenderer()
        vtk_widget.GetRenderWindow().AddRenderer(renderer)
        interactor = vtk_widget.GetRenderWindow().GetInteractor()
        
        # Mouse click logic
        def on_left_button_press(obj, event):
            click_pos = interactor.GetEventPosition()
            print(f"Mouse clicked at: {click_pos}")
        interactor.AddObserver("LeftButtonPressEvent", on_left_button_press)

        # Example 2D object (circle)
        circle = vtk.vtkRegularPolygonSource()
        circle.SetNumberOfSides(50)
        circle.SetRadius(1.0)
        circle.SetCenter(0.0, 0.0, 0.0)

        # Create a PolyData object
        polydata = circle.GetOutput()
    
        # Compute the distance from the center of the circle for each point
        distances = vtk.vtkDoubleArray()
        distances.SetName("Distance")
        for i in range(polydata.GetNumberOfPoints()):
            point = polydata.GetPoint(i)
            distance = vtk.vtkMath.Distance2BetweenPoints(point, [0.0, 0.0, 0.0]) ** 0.5  # Euclidean distance
            distances.InsertNextValue(distance)
    
        # Add distances as a scalar field to the polydata
        polydata.GetPointData().AddArray(distances)
        polydata.GetPointData().SetActiveScalars("Distance")
    
        # Create a color transfer function for the gradient
        color_transfer_function = vtk.vtkColorTransferFunction()
        color_transfer_function.AddRGBPoint(0.0, 1.0, 0.0, 0.0)   # Red for center (Distance = 0)
        color_transfer_function.AddRGBPoint(1.0, 0.0, 1.0, 0.0)   # Green for edge (Distance = 1)
    
        # Create mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        mapper.SetScalarRange(0.0, 1.0)  # Set the range of the scalar values (min, max)
        mapper.SetLookupTable(color_transfer_function)
    
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Add actor and render
        renderer.AddActor(actor)
        renderer.ResetCamera()
        renderer.GetActiveCamera().ParallelProjectionOn() # Set orthographic view
        interactor.Initialize()

        # Configure interactions
        interactor_style = vtk.vtkInteractorStyleRubberBand2D()
        interactor.SetInteractorStyle(interactor_style)

        # Add tab
        self.tabs.addTab(tab, "2D View")

    # Detect close button pressed
    def closeEvent(self, event):
        # Show confirmation dialog
        reply = QMessageBox.question(
            self,
            'Quit Application',
            'Are you sure you want to quit?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Cleanup VTK render windows before exiting
            for i in range(self.tabs.count()):
                tab = self.tabs.widget(i)
                vtk_widgets = tab.findChildren(QVTKRenderWindowInteractor)
                for vtk_widget in vtk_widgets:
                    # Finalize interactor and render window 
                    render_window = vtk_widget.GetRenderWindow()
                    interactor = render_window.GetInteractor()
    
                    if interactor is not None:
                        interactor.TerminateApp()  # Terminate interactor
                    render_window.Finalize()  # Finalize the render window
    
            event.accept()  # Close the application
        else:
            event.ignore()  # Ignore the close event

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())