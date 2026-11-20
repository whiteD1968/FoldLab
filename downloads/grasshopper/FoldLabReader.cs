// Folded Plate Toolkit - Fold Lab reader
// Rhino 8 Grasshopper C# Script component (FOLDLAB). Model units: FEET.
//
// Brings a design from the Fold Lab web page into Rhino so the toolkit's
// fabrication and analysis components can work on it. In the Fold Lab press
// "Copy design" and paste into a Panel wired to Design, or give the path of a
// downloaded .json file. Lengths come in feet, as the toolkit expects.
//
// Inputs:
//   Design : the copied design text (from a Panel), or a path to the .json file
//   Bake   : button; bakes the slab, the folds, the supports, the pillars and the columns
//            to the layer "Fold Lab" and its sublayers
// Outputs:
//   Plates   : the roof as a mesh of its plates (mid-surface). Wire it into the
//              Plates input of STRESS, FORMWORK or TIMBER.
//   Solid    : the slab at its thickness, mitred at the folds
//   Ridges   : ridge folds (lines)
//   Valleys  : valley folds (lines)
//   Supports : the support points (column heads). Wire into STRESS Supports.
//   Pillars  : the pillar legs, head to foot (lines)
//   Columns  : the columns as solids, with the profile and sizes chosen in the Fold Lab
//   Report   : what came in, what was changed to make the plates flat, and what to
//              connect next
//
// Plates that are not flat (a four-sided plate whose corners are not in one plane,
// or a plate of five or more corners) are split into triangles here, so every
// panel can be built flat. The report says how many.
// The model kit needs plates straight from the Fold Designer (it works strip by
// strip along the path), so it does not take a Fold Lab roof.
#region Usings
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using Rhino;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
#endregion

public class Script_Instance : GH_ScriptInstance
{
  // ---------- shared helpers ----------
  private static double Num(object o, double fallback)
  {
    if (o == null) return fallback;
    if (o is GH_Number) return ((GH_Number)o).Value;
    if (o is GH_Integer) return ((GH_Integer)o).Value;
    if (o is string || o is GH_String || o is IGH_Goo)
    {
      // text from a Panel, e.g. "0.3"
      double r;
      string txt = o.ToString().Trim();
      if (double.TryParse(txt, System.Globalization.NumberStyles.Float,
          System.Globalization.CultureInfo.InvariantCulture, out r)) return r;
      return fallback;
    }
    try { return Convert.ToDouble(o); } catch { return fallback; }
  }

  // Mitered solid: every vertex moves to where the offset planes of its
  // neighbouring plates meet, so plates join cleanly at the folds.
  // Vertices listed in "ground" stay at their own height (flat bearing edge).
  private static Mesh Thicken(Mesh m, double t, HashSet<int> ground)
  {
    int nV = m.Vertices.Count;
    double[,] M = new double[nV, 6];   // xx, xy, xz, yy, yz, zz
    double[,] B = new double[nV, 3];
    var faceIdx = new List<int[]>();
    var faceN = new List<Vector3d>();
    for (int f = 0; f < m.Faces.Count; f++)
    {
      MeshFace mf = m.Faces[f];
      int[] idx = mf.IsQuad ? new int[] { mf.A, mf.B, mf.C, mf.D } : new int[] { mf.A, mf.B, mf.C };
      faceIdx.Add(idx);
      Vector3d n = Vector3d.CrossProduct(
        m.Vertices.Point3dAt(idx[1]) - m.Vertices.Point3dAt(idx[0]),
        m.Vertices.Point3dAt(idx[2]) - m.Vertices.Point3dAt(idx[0]));
      faceN.Add(n);
      if (!n.Unitize()) { faceN[faceN.Count - 1] = Vector3d.Zero; continue; }
      faceN[faceN.Count - 1] = n;
      for (int k = 0; k < idx.Length; k++)
      {
        int v = idx[k];
        M[v, 0] += n.X * n.X; M[v, 1] += n.X * n.Y; M[v, 2] += n.X * n.Z;
        M[v, 3] += n.Y * n.Y; M[v, 4] += n.Y * n.Z; M[v, 5] += n.Z * n.Z;
        B[v, 0] += n.X * t / 2.0; B[v, 1] += n.Y * t / 2.0; B[v, 2] += n.Z * t / 2.0;
      }
    }
    // Free edges: hold each corner on the plane of its own edge, so the end of the
    // slab is a clean flat cut and keeps its true thickness. Where two free edges meet
    // at a corner (the zigzag of a section at a free end) that plane is the one through
    // both edges; along a single straight free edge it is square to the plate.
    var edgeUse = new Dictionary<long, int>();
    foreach (int[] idx in faceIdx)
      for (int k = 0; k < idx.Length; k++)
      {
        int p = idx[k], q = idx[(k + 1) % idx.Length];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        int n0; edgeUse.TryGetValue(key, out n0); edgeUse[key] = n0 + 1;
      }
    var freeDirs = new List<Vector3d>[nV];
    var freeNorm = new Vector3d[nV];
    for (int f = 0; f < faceIdx.Count; f++)
    {
      int[] idx = faceIdx[f];
      for (int k = 0; k < idx.Length; k++)
      {
        int p = idx[k], q = idx[(k + 1) % idx.Length];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        if (edgeUse[key] != 1) continue;
        Vector3d e = m.Vertices.Point3dAt(q) - m.Vertices.Point3dAt(p);
        if (!e.Unitize()) continue;
        if (freeDirs[p] == null) freeDirs[p] = new List<Vector3d>();
        if (freeDirs[q] == null) freeDirs[q] = new List<Vector3d>();
        freeDirs[p].Add(e); freeDirs[q].Add(-e);
        freeNorm[p] += faceN[f]; freeNorm[q] += faceN[f];
      }
    }
    for (int v = 0; v < nV; v++)
    {
      if (freeDirs[v] == null) continue;
      Vector3d wn = Vector3d.Zero;
      if (freeDirs[v].Count >= 2) wn = Vector3d.CrossProduct(freeDirs[v][0], freeDirs[v][1]);
      if (wn.Length < 0.05)          // the two free edges run straight on: cut square to the plate
      {
        Vector3d nf = freeNorm[v]; nf.Unitize();
        wn = Vector3d.CrossProduct(freeDirs[v][0], nf);
      }
      if (!wn.Unitize()) continue;
      // Both free edges in the same plate (an outer corner of the roof): the plane
      // through them is the plate itself, and holding the corner in it would squash
      // the slab to nothing. The plate normal alone already gives a square corner.
      { Vector3d nf = freeNorm[v]; if (nf.Unitize() && Math.Abs(wn * nf) > 0.9) continue; }
      double wgt = 100.0;
      M[v, 0] += wgt * wn.X * wn.X; M[v, 1] += wgt * wn.X * wn.Y; M[v, 2] += wgt * wn.X * wn.Z;
      M[v, 3] += wgt * wn.Y * wn.Y; M[v, 4] += wgt * wn.Y * wn.Z; M[v, 5] += wgt * wn.Z * wn.Z;
    }
    var off = new Vector3d[nV];
    double eps = 1e-6;
    for (int v = 0; v < nV; v++)
    {
      double zz = M[v, 5];
      if (ground != null && ground.Contains(v)) zz += 100.0;   // hold the height
      double a = M[v, 0] + eps, b = M[v, 1], c = M[v, 2];
      double d = M[v, 3] + eps, e = M[v, 4], g = zz + eps;
      double r0 = B[v, 0], r1 = B[v, 1], r2 = B[v, 2];
      double det = a * (d * g - e * e) - b * (b * g - e * c) + c * (b * e - d * c);
      if (Math.Abs(det) < 1e-30) { off[v] = Vector3d.Zero; continue; }
      double x = (r0 * (d * g - e * e) - b * (r1 * g - e * r2) + c * (r1 * e - d * r2)) / det;
      double y = (a * (r1 * g - e * r2) - r0 * (b * g - e * c) + c * (b * r2 - r1 * c)) / det;
      double z = (a * (d * r2 - r1 * e) - b * (b * r2 - r1 * c) + r0 * (b * e - d * c)) / det;
      off[v] = new Vector3d(x, y, z);
      // at very sharp folds the exact miter point runs far out and the solid
      // turns inside out; cap it at four half-thicknesses (a blunt miter)
      double lim = 4.0 * t;
      double ol = off[v].Length;
      if (ol > lim) off[v] = off[v] * (lim / ol);
    }

    Mesh s = new Mesh();
    for (int v = 0; v < nV; v++) s.Vertices.Add(m.Vertices.Point3dAt(v) - off[v]);
    for (int v = 0; v < nV; v++) s.Vertices.Add(m.Vertices.Point3dAt(v) + off[v]);
    var count = new Dictionary<long, int>();
    foreach (int[] idx in faceIdx)
    {
      int c = idx.Length;
      if (c == 4)
      {
        s.Faces.AddFace(idx[3], idx[2], idx[1], idx[0]);
        s.Faces.AddFace(idx[0] + nV, idx[1] + nV, idx[2] + nV, idx[3] + nV);
      }
      else
      {
        s.Faces.AddFace(idx[2], idx[1], idx[0]);
        s.Faces.AddFace(idx[0] + nV, idx[1] + nV, idx[2] + nV);
      }
      for (int k = 0; k < c; k++)
      {
        int p = idx[k], q = idx[(k + 1) % c];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        int n0; count.TryGetValue(key, out n0); count[key] = n0 + 1;
      }
    }
    foreach (int[] idx in faceIdx)
    {
      int c = idx.Length;
      for (int k = 0; k < c; k++)
      {
        int p = idx[k], q = idx[(k + 1) % c];
        long key = (long)Math.Min(p, q) * nV + Math.Max(p, q);
        if (count[key] == 1) s.Faces.AddFace(p, q, q + nV, p + nV);   // edge wall
      }
    }
    s.Compact();
    s.Unweld(0, true);            // flat-shade every plate so folds read as crisp edges
    s.Normals.ComputeNormals();
    return s;
  }

  // Sort interior creases into ridges (fold points toward the face normals)
  // and valleys. Coplanar edges are ignored.
  private static void Creases(Mesh m, List<Line> ridges, List<Line> valleys)
  {
    double tol = 1e-6;
    for (int e = 0; e < m.TopologyEdges.Count; e++)
    {
      int[] fs = m.TopologyEdges.GetConnectedFaces(e);
      if (fs == null || fs.Length != 2) continue;
      Line ln = m.TopologyEdges.EdgeLine(e);
      MeshFace f1 = m.Faces[fs[0]];
      Point3d a = m.Vertices.Point3dAt(f1.A);
      Point3d b = m.Vertices.Point3dAt(f1.B);
      Point3d c = m.Vertices.Point3dAt(f1.C);
      Vector3d n1 = Vector3d.CrossProduct(b - a, c - a);
      if (!n1.Unitize()) continue;
      MeshFace f2 = m.Faces[fs[1]];
      int[] idx = f2.IsQuad ? new int[] { f2.A, f2.B, f2.C, f2.D } : new int[] { f2.A, f2.B, f2.C };
      double best = 0.0; Point3d far = ln.From;
      for (int k = 0; k < idx.Length; k++)
      {
        Point3d q = m.Vertices.Point3dAt(idx[k]);
        double d = ln.DistanceTo(q, false);
        if (d > best) { best = d; far = q; }
      }
      double s = n1 * (far - ln.From);
      if (s < -tol * Math.Max(1.0, best)) ridges.Add(ln);
      else if (s > tol * Math.Max(1.0, best)) valleys.Add(ln);
    }
  }

  private static double AreaOf(Mesh m)
  {
    AreaMassProperties amp = AreaMassProperties.Compute(m);
    return amp == null ? 0.0 : amp.Area;
  }

  // The toolkit reads every length in the model as feet. Warn if the file is not in feet.
  private static string UnitNote()
  {
    RhinoDoc doc = RhinoDoc.ActiveDoc;
    if (doc == null || doc.ModelUnitSystem == UnitSystem.Feet) return null;
    return "This Rhino file is in " + doc.ModelUnitSystem.ToString().ToLower()
      + ", but the toolkit reads every length as feet. Set the file to feet (Units command, do not scale the model) or the sizes, slab thickness and quantities shown here are wrong.";
  }

  // ---------- a small JSON reader (objects, arrays, numbers, strings, true/false/null) ----------
  private class Json
  {
    private readonly string s; private int i;
    public Json(string text) { s = text; i = 0; }
    public static object Parse(string text) { var j = new Json(text); j.Ws(); object v = j.Value(); return v; }
    private void Ws() { while (i < s.Length && char.IsWhiteSpace(s[i])) i++; }
    private object Value()
    {
      Ws();
      if (i >= s.Length) throw new Exception("the text ends too early");
      char c = s[i];
      if (c == '{') return Obj();
      if (c == '[') return Arr();
      if (c == '"') return Str();
      if (c == 't' && Peek("true")) { i += 4; return true; }
      if (c == 'f' && Peek("false")) { i += 5; return false; }
      if (c == 'n' && Peek("null")) { i += 4; return null; }
      return NumberValue();
    }
    private bool Peek(string w) { return string.CompareOrdinal(s, i, w, 0, w.Length) == 0; }
    private Dictionary<string, object> Obj()
    {
      var d = new Dictionary<string, object>(); i++; Ws();
      if (i < s.Length && s[i] == '}') { i++; return d; }
      while (true)
      {
        Ws(); string k = Str(); Ws();
        if (i >= s.Length || s[i] != ':') throw new Exception("a ':' is missing near character " + i);
        i++; d[k] = Value(); Ws();
        if (i < s.Length && s[i] == ',') { i++; continue; }
        if (i < s.Length && s[i] == '}') { i++; return d; }
        throw new Exception("a ',' or '}' is missing near character " + i);
      }
    }
    private List<object> Arr()
    {
      var a = new List<object>(); i++; Ws();
      if (i < s.Length && s[i] == ']') { i++; return a; }
      while (true)
      {
        a.Add(Value()); Ws();
        if (i < s.Length && s[i] == ',') { i++; continue; }
        if (i < s.Length && s[i] == ']') { i++; return a; }
        throw new Exception("a ',' or ']' is missing near character " + i);
      }
    }
    private string Str()
    {
      if (s[i] != '"') throw new Exception("text in quotes was expected near character " + i);
      i++; var sb = new StringBuilder();
      while (i < s.Length && s[i] != '"')
      {
        if (s[i] == '\\' && i + 1 < s.Length)
        {
          char e = s[i + 1]; i += 2;
          if (e == 'n') sb.Append('\n'); else if (e == 't') sb.Append('\t');
          else if (e == 'u' && i + 4 <= s.Length) { sb.Append((char)Convert.ToInt32(s.Substring(i, 4), 16)); i += 4; }
          else sb.Append(e);
        }
        else sb.Append(s[i++]);
      }
      i++; return sb.ToString();
    }
    private double NumberValue()
    {
      int j = i;
      while (i < s.Length && "+-0123456789.eE".IndexOf(s[i]) >= 0) i++;
      double v;
      if (!double.TryParse(s.Substring(j, i - j), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out v))
        throw new Exception("a number could not be read near character " + j);
      return v;
    }
  }
  private static double D(object o) { return o is double ? (double)o : Convert.ToDouble(o, System.Globalization.CultureInfo.InvariantCulture); }
  private static Point3d P3(object o) { var a = (List<object>)o; return new Point3d(D(a[0]), D(a[1]), D(a[2])); }

  // the column section, as in the Fold Lab: square, round, cruciform or a 3 : 1 blade; D in ft
  private static List<double[]> ColOutline(string profile, double D)
  {
    var o = new List<double[]>(); double r = D / 2;
    if (profile.StartsWith("round")) { for (int k = 0; k < 16; k++) o.Add(new[] { r * Math.Cos(k * Math.PI / 8), r * Math.Sin(k * Math.PI / 8) }); }
    else if (profile.StartsWith("cruciform"))
    { double w = D / 8; double[,] p = { { r, w }, { w, w }, { w, r }, { -w, r }, { -w, w }, { -r, w }, { -r, -w }, { -w, -w }, { -w, -r }, { w, -r }, { w, -w }, { r, -w } };
      for (int k = 0; k < 12; k++) o.Add(new[] { p[k, 0], p[k, 1] }); }
    else if (profile.StartsWith("blade")) { double h = D / 6; o.Add(new[] { r, h }); o.Add(new[] { -r, h }); o.Add(new[] { -r, -h }); o.Add(new[] { r, -h }); }
    else { o.Add(new[] { r, r }); o.Add(new[] { -r, r }); o.Add(new[] { -r, -r }); o.Add(new[] { r, -r }); }
    return o;
  }

  private bool lastBake = false;

  private void RunScript(object Design, object Bake,
    ref object Plates, ref object Solid, ref object Ridges, ref object Valleys,
    ref object Supports, ref object Pillars, ref object Columns, ref object Report)
  {
    var sb = new StringBuilder();
    string text = Design == null ? "" : Design.ToString().Trim();
    if (text.Length == 0) { Report = "Paste the design from the Fold Lab (Copy design) into a Panel and wire it to Design, or give the path of the .json file."; return; }

    // a path to a file, or the text itself
    string from = "pasted text";
    if (!text.StartsWith("{"))
    {
      string path = text.Trim('"');
      if (!System.IO.File.Exists(path)) { Report = "No file at " + path + ". Give the full path of the .json file, or paste the copied design instead."; return; }
      text = System.IO.File.ReadAllText(path); from = System.IO.Path.GetFileName(path);
    }

    Dictionary<string, object> doc;
    try { doc = Json.Parse(text) as Dictionary<string, object>; }
    catch (Exception ex) { Report = "This is not a complete Fold Lab design: " + ex.Message + ". Copy it again from the Fold Lab and paste the whole text."; return; }
    if (doc == null || !doc.ContainsKey("nodes") || !doc.ContainsKey("plates")) { Report = "This text has no nodes and plates. Use Copy design in the Fold Lab."; return; }

    // ---------------- nodes, plates, supports ----------------
    var nodes = new List<Point3d>();
    foreach (object o in (List<object>)doc["nodes"]) nodes.Add(P3(o));
    var plates = new List<int[]>();
    foreach (object o in (List<object>)doc["plates"])
    {
      var a = (List<object>)o; var q = new int[a.Count];
      for (int k = 0; k < a.Count; k++) q[k] = (int)Math.Round(D(a[k]));
      plates.Add(q);
    }
    var supIds = new List<int>();
    if (doc.ContainsKey("supports") && doc["supports"] is List<object>)
      foreach (object o in (List<object>)doc["supports"]) supIds.Add((int)Math.Round(D(o)));
    var legs = new List<Line>();
    if (doc.ContainsKey("pillars") && doc["pillars"] is List<object>)
      foreach (object o in (List<object>)doc["pillars"])
      {
        var pd = o as Dictionary<string, object>; if (pd == null) continue;
        int h = (int)Math.Round(D(pd["head"]));
        if (h >= 0 && h < nodes.Count) legs.Add(new Line(nodes[h], P3(pd["foot"])));
      }
    double thkIn = 4.0;
    string source = doc.ContainsKey("source") && doc["source"] != null ? doc["source"].ToString() : "Fold Lab";
    var par = doc.ContainsKey("parameters") ? doc["parameters"] as Dictionary<string, object> : null;
    if (par != null && par.ContainsKey("thk")) thkIn = D(par["thk"]);
    double tFt = thkIn / 12.0;

    // ---------------- columns: the profile and sizes chosen in the Fold Lab ----------------
    string colProfile = "square"; double footIn = 12, headIn = 12; bool alongSpan = true;
    var cd = doc.ContainsKey("columns") ? doc["columns"] as Dictionary<string, object> : null;
    if (cd != null)
    {
      if (cd.ContainsKey("profile") && cd["profile"] != null) colProfile = cd["profile"].ToString();
      if (cd.ContainsKey("footIn")) footIn = D(cd["footIn"]);
      if (cd.ContainsKey("headIn")) headIn = D(cd["headIn"]);
      if (cd.ContainsKey("longSide") && cd["longSide"] != null) alongSpan = !cd["longSide"].ToString().Contains("across");
    }
    var columns = new List<Mesh>();
    foreach (Line lg in legs)
    {
      Point3d head = lg.From - new Vector3d(0, 0, tFt / 2), foot = lg.To;
      Vector3d ax = head - foot; if (ax.Length < 1e-6) continue; ax.Unitize();
      Vector3d d0 = alongSpan ? Vector3d.YAxis : Vector3d.XAxis;
      Vector3d e1 = d0 - ax * (d0 * ax); if (e1.Length < 1e-3) e1 = Vector3d.XAxis; e1.Unitize();
      Vector3d e2 = Vector3d.CrossProduct(ax, e1);
      var oF = ColOutline(colProfile, footIn / 12.0); var oH = ColOutline(colProfile, headIn / 12.0);
      var cm = new Mesh(); int nn = oF.Count;
      for (int k = 0; k < nn; k++) cm.Vertices.Add(foot + e1 * oF[k][0] + e2 * oF[k][1]);
      for (int k = 0; k < nn; k++) cm.Vertices.Add(head + e1 * oH[k][0] + e2 * oH[k][1]);
      for (int k = 0; k < nn; k++) { int k2 = (k + 1) % nn; cm.Faces.AddFace(k, k2, nn + k2, nn + k); }
      int cF = cm.Vertices.Add(foot), cH = cm.Vertices.Add(head);
      for (int k = 0; k < nn; k++) { int k2 = (k + 1) % nn; cm.Faces.AddFace(cF, k2, k); cm.Faces.AddFace(cH, nn + k, nn + k2); }
      cm.Normals.ComputeNormals(); cm.Compact();
      columns.Add(cm);
    }

    // ---------------- mesh: flat plates; warped quads and larger plates split into triangles ----------------
    var m = new Mesh();
    foreach (Point3d p in nodes) m.Vertices.Add(p);
    int split4 = 0, splitN = 0, bad = 0;
    Action<int, int, int> tri = (a, b, c) =>
    {
      Vector3d n = Vector3d.CrossProduct(nodes[b] - nodes[a], nodes[c] - nodes[a]);
      if (n.Length < 1e-9) { bad++; return; }
      if (n.Z < 0) m.Faces.AddFace(a, c, b); else m.Faces.AddFace(a, b, c);   // every plate faces up
    };
    foreach (int[] q in plates)
    {
      bool okIds = true; foreach (int v in q) if (v < 0 || v >= nodes.Count) okIds = false;
      if (!okIds || q.Length < 3) { bad++; continue; }
      if (q.Length == 3) { tri(q[0], q[1], q[2]); continue; }
      if (q.Length == 4)
      {
        Plane pl; Plane.FitPlaneToPoints(new Point3d[] { nodes[q[0]], nodes[q[1]], nodes[q[2]], nodes[q[3]] }, out pl);
        double warp = 0; foreach (int v in q) warp = Math.Max(warp, Math.Abs(pl.DistanceTo(nodes[v])));
        double size = Math.Max(nodes[q[0]].DistanceTo(nodes[q[2]]), nodes[q[1]].DistanceTo(nodes[q[3]]));
        if (warp <= 1e-3 * Math.Max(1.0, size))
        {
          Vector3d n = Vector3d.CrossProduct(nodes[q[2]] - nodes[q[0]], nodes[q[3]] - nodes[q[1]]);
          if (n.Z < 0) m.Faces.AddFace(q[0], q[3], q[2], q[1]); else m.Faces.AddFace(q[0], q[1], q[2], q[3]);
          continue;
        }
        split4++;
        // split on the shorter diagonal
        if (nodes[q[0]].DistanceTo(nodes[q[2]]) <= nodes[q[1]].DistanceTo(nodes[q[3]])) { tri(q[0], q[1], q[2]); tri(q[0], q[2], q[3]); }
        else { tri(q[0], q[1], q[3]); tri(q[1], q[2], q[3]); }
        continue;
      }
      // five or more corners: a fan from the corner that leaves no sliver
      splitN++;
      int best = 0; double bestMin = -1;
      for (int s0 = 0; s0 < q.Length; s0++)
      {
        double mn = double.MaxValue;
        for (int k = 1; k + 1 < q.Length; k++)
        {
          Point3d A = nodes[q[s0]], B = nodes[q[(s0 + k) % q.Length]], C = nodes[q[(s0 + k + 1) % q.Length]];
          double area2 = Vector3d.CrossProduct(B - A, C - A).Length;
          mn = Math.Min(mn, area2 / Math.Max(1e-12, A.DistanceTo(B) * A.DistanceTo(C) + B.DistanceTo(C) * B.DistanceTo(C)));
        }
        if (mn > bestMin) { bestMin = mn; best = s0; }
      }
      for (int k = 1; k + 1 < q.Length; k++) tri(q[best], q[(best + k) % q.Length], q[(best + k + 1) % q.Length]);
    }
    m.Vertices.CombineIdentical(true, true);
    m.Compact();
    m.Normals.ComputeNormals();
    m.SetUserString("FoldToolkit.SlabFt", tFt.ToString(System.Globalization.CultureInfo.InvariantCulture));   // STRESS reads the slab from the roof itself

    var ridges = new List<Line>(); var valleys = new List<Line>();
    Creases(m, ridges, valleys);
    Mesh solid = null;
    try { solid = Thicken(m, tFt, new HashSet<int>()); } catch { solid = null; }
    var sup = new List<Point3d>();
    foreach (int id in supIds) if (id >= 0 && id < nodes.Count) sup.Add(nodes[id]);

    // share with STRESS and the window: slab thickness and supports
    try
    {
      AppDomain.CurrentDomain.SetData("FoldToolkit.SlabFt", tFt);
      AppDomain.CurrentDomain.SetData("FoldToolkit.SupportPts", sup.ToArray());
    }
    catch { }

    // ---------------- bake ----------------
    bool bakeNow = Bake is bool ? (bool)Bake : (Bake is GH_Boolean && ((GH_Boolean)Bake).Value);
    if (bakeNow && !lastBake && RhinoDoc.ActiveDoc != null)
    {
      RhinoDoc rdoc = RhinoDoc.ActiveDoc;
      int parent = rdoc.Layers.FindByFullPath("Fold Lab", -1);
      if (parent < 0) parent = rdoc.Layers.Add(new Rhino.DocObjects.Layer { Name = "Fold Lab" });
      string[] names = { "Slab", "Ridges", "Valleys", "Supports", "Pillars", "Columns" };
      System.Drawing.Color[] cols = { System.Drawing.Color.FromArgb(150, 150, 150), System.Drawing.Color.FromArgb(20, 20, 20),
        System.Drawing.Color.FromArgb(20, 110, 120), System.Drawing.Color.FromArgb(20, 20, 20), System.Drawing.Color.FromArgb(90, 90, 90),
        System.Drawing.Color.FromArgb(120, 120, 120) };
      var atts = new Rhino.DocObjects.ObjectAttributes[names.Length];
      for (int L = 0; L < names.Length; L++)
      {
        int li = rdoc.Layers.FindByFullPath("Fold Lab::" + names[L], -1);
        if (li < 0) li = rdoc.Layers.Add(new Rhino.DocObjects.Layer { Name = names[L], ParentLayerId = rdoc.Layers[parent].Id, Color = cols[L] });
        atts[L] = new Rhino.DocObjects.ObjectAttributes { LayerIndex = li };
      }
      if (solid != null) rdoc.Objects.AddMesh(solid, atts[0]); else rdoc.Objects.AddMesh(m, atts[0]);
      foreach (Line l in ridges) rdoc.Objects.AddLine(l, atts[1]);
      foreach (Line l in valleys) rdoc.Objects.AddLine(l, atts[2]);
      foreach (Point3d p in sup) rdoc.Objects.AddPoint(p, atts[3]);
      foreach (Line l in legs) rdoc.Objects.AddLine(l, atts[4]);
      foreach (Mesh cmb in columns) rdoc.Objects.AddMesh(cmb, atts[5]);
      rdoc.Views.Redraw();
      sb.AppendLine("Baked to the layer Fold Lab.");
    }
    lastBake = bakeNow;

    // ---------------- report ----------------
    BoundingBox bb = m.GetBoundingBox(true);
    sb.AppendLine("FOLD LAB DESIGN");
    sb.AppendLine(string.Format("  From                {0} ({1})", from, source));
    sb.AppendLine(string.Format("  Plates              {0} in the design, {1} mesh faces", plates.Count, m.Faces.Count));
    sb.AppendLine(string.Format("  Plan                {0:0.0} x {1:0.0} ft, height {2:0.0} to {3:0.0} ft", bb.Max.X - bb.Min.X, bb.Max.Y - bb.Min.Y, bb.Min.Z, bb.Max.Z));
    sb.AppendLine(string.Format("  Slab                {0:0.0} in (published for STRESS)", thkIn));
    sb.AppendLine(string.Format("  Folds               {0} ridges, {1} valleys", ridges.Count, valleys.Count));
    sb.AppendLine(string.Format("  Supports            {0} points, {1} pillar legs (published for STRESS)", sup.Count, legs.Count));
    sb.AppendLine(string.Format("  Columns             {0}, {1:0} in at the foot, {2:0} in at the head, long side {3}", colProfile, footIn, headIn, alongSpan ? "along the span" : "across the span"));
    sb.AppendLine();
    sb.AppendLine("FLAGS");
    if (split4 > 0) sb.AppendLine(string.Format("  - {0} four-sided plates were not flat (their corners are not in one plane) and were split into two triangles each, so every panel can be built flat.", split4));
    if (splitN > 0) sb.AppendLine(string.Format("  - {0} plates with five or more corners were split into triangles.", splitN));
    if (bad > 0) sb.AppendLine(string.Format("  - {0} plates or triangles had no area or pointed at missing nodes and were left out.", bad));
    if (solid == null) sb.AppendLine("  - The slab solid could not be made; the mesh of plates is still fine for the other components.");
    string un = UnitNote(); if (un != null) sb.AppendLine("  - " + un);
    sb.AppendLine("  - Next: wire Plates into STRESS (and Supports into its Supports), FORMWORK or TIMBER. The model kit needs plates from the Fold Designer.");
    sb.AppendLine("  - If the Fold Designer is also on the canvas, its run clears the shared supports; wiring Supports into STRESS directly avoids that.");

    Plates = m; Solid = solid; Ridges = ridges; Valleys = valleys; Supports = sup; Pillars = legs; Columns = columns;
    Report = sb.ToString();
  }
}
