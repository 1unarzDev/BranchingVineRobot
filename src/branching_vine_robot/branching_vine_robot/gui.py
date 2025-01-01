
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
        
def add_mouse_listeners(self):
    # Loop through all tabs
    for i in range(self.tabs.count()):
        tab = self.tabs.widget(i)

        # Find all VTK widgets in the tab
        vtk_widgets = tab.findChildren(QVTKRenderWindowInteractor)
        for vtk_widget in vtk_widgets:
            interactor = vtk_widget.GetRenderWindow().GetInteractor()

            # Add Mouse Event Observers
            interactor.AddObserver("MouseMoveEvent", self.mouse_move_event)
            interactor.AddObserver("LeftButtonPressEvent", self.left_click_event)

def mouse_move_event(self, obj, event):
    # Capture mouse position
    x, y = obj.GetEventPosition()
    print(f"Mouse moved to: ({x}, {y})")

def left_click_event(self, obj, event):
    # Capture mouse position on left click
    x, y = obj.GetEventPosition()
    print(f"Left click at: ({x}, {y})")

    # Convert to world coordinates
    renderer = obj.GetRenderWindow().GetRenderers().GetFirstRenderer()
    picker = vtk.vtkPropPicker()
    picker.Pick(x, y, 0, renderer)
    world_pos = picker.GetPickPosition()
    print(f"World coordinates: {world_pos}")

    # Redraw
    obj.GetRenderWindow().Render()

    def add_3d_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Create VTK Widget for 3D
        vtk_widget = QVTKRenderWindowInteractor(tab)
        layout.addWidget(vtk_widget)
        tab.setLayout(layout)

        # Renderer and RenderWindow for 3D
        renderer = vtk.vtkRenderer()
        vtk_widget.GetRenderWindow().AddRenderer(renderer)
        interactor = vtk_widget.GetRenderWindow().GetInteractor()

        # Example 3D object (sphere)
        sphere = vtk.vtkSphereSource()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
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

        # Renderer and RenderWindow for 2D
        renderer = vtk.vtkRenderer()
        vtk_widget.GetRenderWindow().AddRenderer(renderer)
        interactor = vtk_widget.GetRenderWindow().GetInteractor()

        # Example 2D object (circle)
        circle = vtk.vtkRegularPolygonSource()
        circle.SetNumberOfSides(50)
        circle.SetRadius(1.0)
        circle.SetCenter(0.0, 0.0, 0.0)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(circle.GetOutputPort())
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

    # Detect Close Button Pressed
    def closeEvent(self, event):
        # Show confirmation dialog
        reply = QMessageBox.question(
            self,
            'Exit Application',
            'Are you sure you want to exit?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Cleanup VTK render windows before exiting
            for i in range(self.tabs.count()):
                tab = self.tabs.widget(i)
                vtk_widgets = tab.findChildren(QVTKRenderWindowInteractor)
                for vtk_widget in vtk_widgets:
                    # Finalize interactor and render window properly
                    render_window = vtk_widget.GetRenderWindow()
                    interactor = render_window.GetInteractor()
    
                    if interactor is not None:
                        interactor.TerminateApp()  # Properly terminate interactor
                    render_window.Finalize()  # Finalize the render window
    
            event.accept()  # Close the application
        else:
            event.ignore()  # Ignore the close event

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())