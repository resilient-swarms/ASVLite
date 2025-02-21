package vtk.rendering.swt;

import java.awt.Label;

import vtk.rendering.vtkComponent;
import vtk.rendering.vtkInteractorForwarder;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.events.MouseMoveListener;
import org.eclipse.swt.events.MouseTrackListener;
import org.eclipse.swt.events.MouseWheelListener;

/**
 * Decorator class used to implement all Mouse/Key SWT listener and convert them
 * into the vtkInteractorForwarder proper AWT event.
 *
 * @author    Joachim Pouderoux - joachim.pouderoux@kitware.com, Kitware SAS 2012
 * @copyright This work was supported by CEA/CESTA
 *            Commissariat a l'Energie Atomique et aux Energies Alternatives,
 *            15 avenue des Sablieres, CS 60001, 33116 Le Barp, France.
 */
public class vtkSwtInteractorForwarderDecorator extends vtkInteractorForwarder
implements MouseListener, MouseMoveListener, MouseTrackListener, MouseWheelListener, KeyListener {

  vtkInteractorForwarder forwarder;
  Label dummyComponent;

  public vtkSwtInteractorForwarderDecorator(vtkComponent<?> component, vtkInteractorForwarder forwarder) {
    super(component);
    dummyComponent = new Label();
    this.forwarder = forwarder;
  }

  public static int convertModifiers(int mods) {
    int modifiers = 0;
    if ((mods & SWT.SHIFT) != 0) modifiers |= java.awt.Event.SHIFT_DOWN_MASK;
    if ((mods & SWT.CTRL) != 0) modifiers |= java.awt.Event.CTRL_DOWN_MASK;
    if ((mods & SWT.ALT) != 0) modifiers |= java.awt.Event.ALT_DOWN_MASK;
    return modifiers;
  }

  public java.awt.event.KeyEvent convertKeyEvent(org.eclipse.swt.events.KeyEvent e) {
    return new java.awt.event.KeyEvent(dummyComponent, 0, (long)e.time, convertModifiers(e.stateMask), e.keyCode, e.character);
  }

  public java.awt.event.MouseEvent convertMouseEvent(org.eclipse.swt.events.MouseEvent e) {
    int button = 0;
    if ((e.button == 1) || (e.stateMask & SWT.BUTTON1) != 0) button = java.awt.event.MouseEvent.BUTTON1;
    else if ((e.button == 2) || (e.stateMask & SWT.BUTTON2) != 0) button = java.awt.event.MouseEvent.BUTTON2;
    else if ((e.button == 3) || (e.stateMask & SWT.BUTTON3) != 0) button = java.awt.event.MouseEvent.BUTTON3;
    return new java.awt.event.MouseEvent(dummyComponent, 0, (long)e.time, convertModifiers(e.stateMask), e.x, e.y, e.count, false, button);
  }

  public java.awt.event.MouseWheelEvent convertMouseWheelEvent(org.eclipse.swt.events.MouseEvent e) {
    return new java.awt.event.MouseWheelEvent(dummyComponent, 0, e.time, convertModifiers(e.stateMask), e.x, e.y, 0, false, java.awt.event.MouseWheelEvent.WHEEL_UNIT_SCROLL, 1, e.count);
  }

  public void keyPressed(KeyEvent e) {
    super.keyPressed(convertKeyEvent(e));
  }

  public void keyReleased(KeyEvent e) {
    super.keyReleased(convertKeyEvent(e));
  }

  public void mouseEnter(MouseEvent e) {
    super.mouseEntered(convertMouseEvent(e));
  }

  public void mouseExit(MouseEvent e) {
    super.mouseExited(convertMouseEvent(e));
  }

  public void mouseMove(MouseEvent e) {
    if (((e.stateMask & SWT.BUTTON1) == 0)
        && ((e.stateMask & SWT.BUTTON2) == 0)
        && ((e.stateMask & SWT.BUTTON3) == 0)) {
      super.mouseMoved(convertMouseEvent(e));
    } else {
      super.mouseDragged(convertMouseEvent(e));
    }
  }

  public void mouseDown(MouseEvent e) {
    super.mousePressed(convertMouseEvent(e));
  }

  public void mouseUp(MouseEvent e) {
    super.mouseReleased(convertMouseEvent(e));
  }

  public void mouseScrolled(MouseEvent e) {
    super.mouseWheelMoved(convertMouseWheelEvent(e));
  }

  public void mouseHover(MouseEvent e) {
  }

  public void mouseDoubleClick(MouseEvent e) {
  }
}
