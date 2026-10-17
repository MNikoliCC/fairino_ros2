"""Render the rail assembly with VTK (EGL offscreen). usage: render_asm.py tools_dir zip out_prefix"""
import sys, os
os.environ.setdefault("VTK_DEFAULT_OPENGL_WINDOW", "vtkEGLRenderWindow")
sys.path.insert(0, sys.argv[1])
import gen_rail as g, vtk

def polydata(shape, tol):
    v, t = shape.tessellate(tol, 0.4)
    pts = vtk.vtkPoints()
    for p in v: pts.InsertNextPoint(p.x, p.y, p.z)
    cells = vtk.vtkCellArray()
    for a, b, c in t:
        cells.InsertNextCell(3); cells.InsertCellPoint(a); cells.InsertCellPoint(b); cells.InsertCellPoint(c)
    pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(cells)
    n = vtk.vtkPolyDataNormals(); n.SetInputData(pd); n.SetFeatureAngle(35); n.Update()
    return n.GetOutput()

def actors(asm, tol=0.4):
    out = []
    for code, name, shape, rgb in asm.parts:
        m = vtk.vtkPolyDataMapper(); m.SetInputData(polydata(shape, tol))
        a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*rgb)
        a.GetProperty().SetSpecular(0.2); out.append(a)
    return out

def shot(acts, path, pos, focal, up=(0, 0, 1), size=(1400, 900), angle=30, ortho=None):
    rw = vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.SetSize(*size)
    r = vtk.vtkRenderer(); r.SetBackground(1, 1, 1); rw.AddRenderer(r)
    for a in acts: r.AddActor(a)
    cam = r.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
    cam.SetViewAngle(angle)
    if ortho: cam.ParallelProjectionOn(); cam.SetParallelScale(ortho)
    r.ResetCameraClippingRange(); rw.SetMultiSamples(8); rw.Render()
    w = vtk.vtkWindowToImageFilter(); w.SetInput(rw); w.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(path); wr.SetInputConnection(w.GetOutputPort()); wr.Write()

if __name__ == "__main__":
    cat = g.Catalog(sys.argv[2]); asm = g.build(cat); pre = sys.argv[3]
    acts = actors(asm)
    zc = 200
    shot(acts, pre + "_iso.png", (2300, -900, 1500), (0, 1450, 150), angle=33, size=(1600, 1000))
    shot(acts, pre + "_start.png", (1000, -650, 650), (0, 300, 200), angle=32)
    shot(acts, pre + "_endleg.png", (-150, 420, 20), (230, 40, 150), angle=40)
    shot(acts, pre + "_foot.png", (650, 700, 120), (272, 1010, 120), angle=30)
    shot(acts, pre + "_handle.png", (900, 250, 420), (295, 515, 260), angle=30)
    print("rendered")
