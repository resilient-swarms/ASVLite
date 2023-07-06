package vtk;

import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

/**
 * vtkRenderWindowPanel is a vtkCanvas which allows additional vtkRenderers to
 * be added. vtkPanel and vtkCanvas force you to add actors to the internal
 * vtkRenderer. vtkRenderWindowPanel always renders, even if the internal
 * renderer has no visible actors.
 *
 * @author Kitware
 */
public class vtkRenderWindowPanel extends vtkCanvas {
  private static final long serialVersionUID = 1L;

  public vtkRenderWindowPanel() {
    cam = new vtkCamera();
    lgt = new vtkLight();
  }

  public vtkRenderWindowPanel(vtkRenderWindow win) {
    super(win);
    cam = new vtkCamera();
    lgt = new vtkLight();
  }

  public synchronized void Render() {
    if (!rendering) {
      rendering = true;
      if (rw != null) {
        if (windowset == 0) {
          // set the window id and the active camera
          RenderCreate(rw);
          Lock();
          rw.SetSize(getWidth(), getHeight());
          UnLock();
          windowset = 1;
          // notify observers that we have a renderwindow created
          // windowSetObservable.notifyObservers();
        }
        Lock();
        rw.Render();
        UnLock();
      }
      rendering = false;
    }
  }

  public void mousePressed(MouseEvent e) {
    Lock();
    rw.SetDesiredUpdateRate(5.0);
    lastX = e.getX();
    lastY = e.getY();

    ctrlPressed = (e.getModifiersEx() & InputEvent.CTRL_DOWN_MASK) == InputEvent.CTRL_DOWN_MASK ? 1 : 0;
    shiftPressed = (e.getModifiersEx() & InputEvent.SHIFT_DOWN_MASK) == InputEvent.SHIFT_DOWN_MASK ? 1 : 0;

    iren.SetEventInformationFlipY(e.getX(), e.getY(), ctrlPressed, shiftPressed, '0', 0, "0");

    if ((e.getModifiersEx() & InputEvent.BUTTON1_DOWN_MASK) == InputEvent.BUTTON1_DOWN_MASK) {
      iren.LeftButtonPressEvent();
    }

    else if ((e.getModifiersEx() & InputEvent.BUTTON2_DOWN_MASK) == InputEvent.BUTTON2_DOWN_MASK) {
      iren.MiddleButtonPressEvent();
    }

    else if ((e.getModifiersEx() & InputEvent.BUTTON3_DOWN_MASK) == InputEvent.BUTTON3_DOWN_MASK) {
      iren.RightButtonPressEvent();
    }

    UnLock();
  }

  public void mouseDragged(MouseEvent e) {
    ctrlPressed = (e.getModifiersEx() & InputEvent.CTRL_DOWN_MASK) == InputEvent.CTRL_DOWN_MASK ? 1 : 0;
    shiftPressed = (e.getModifiersEx() & InputEvent.SHIFT_DOWN_MASK) == InputEvent.SHIFT_DOWN_MASK ? 1 : 0;

    iren.SetEventInformationFlipY(e.getX(), e.getY(), ctrlPressed, shiftPressed, '0', 0, "0");

    Lock();
    iren.MouseMoveEvent();
    UnLock();
  }

  public void mouseWheelMoved(MouseWheelEvent e) {
    ctrlPressed = (e.getModifiersEx() & InputEvent.CTRL_DOWN_MASK) == InputEvent.CTRL_DOWN_MASK ? 1 : 0;
    shiftPressed = (e.getModifiersEx() & InputEvent.SHIFT_DOWN_MASK) == InputEvent.SHIFT_DOWN_MASK ? 1 : 0;

    Lock();
    if (e.getWheelRotation() > 0) {
      iren.SetEventInformationFlipY(e.getX(), e.getY(), ctrlPressed, shiftPressed, '0', 0, "0");
      iren.MouseWheelBackwardEvent();
    }
    else if (e.getWheelRotation() < 0) {
      iren.SetEventInformationFlipY(e.getX(), e.getY(), ctrlPressed, shiftPressed, '0', 0, "0");
      iren.MouseWheelForwardEvent();
    }
    UnLock();
  }

  public void keyPressed(KeyEvent e) {
    char keyChar = e.getKeyChar();

    ctrlPressed = (e.getModifiersEx() & InputEvent.CTRL_DOWN_MASK) == InputEvent.CTRL_DOWN_MASK ? 1 : 0;
    shiftPressed = (e.getModifiersEx() & InputEvent.SHIFT_DOWN_MASK) == InputEvent.SHIFT_DOWN_MASK ? 1 : 0;

    iren.SetEventInformationFlipY(lastX, lastY, ctrlPressed, shiftPressed, keyChar, 0, String.valueOf(keyChar));

    Lock();
    iren.KeyPressEvent();
    iren.CharEvent();
    UnLock();
  }
}
