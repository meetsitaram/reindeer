"""Build the reindeer in Fusion from spec/robot.json (the spec is the source of truth: never hand-edit the document;
change spec/robot.py, rerun the checks, rebuild). figlib.py, duck-droids' (Apache-2.0), turns each spec shape into a body.

One short job per MCP call:
  build.new_document()        a new Z-up design with no timeline (Direct): bodies go in fast
  build.link('head')          one link as its own component at the zero pose: each geom a body, its cuts applied
  build.parametric()          after the last link: the timeline on, which joints need
  build.joints()              as-built revolute joints at the spec's origins, with its limits
  build.status()              links, joints, body counts against the spec, unhealthy features
  build.render(path, view)    an orthographic picture, reporting whether the file was written
  build.save(name)            into the hub: my-projects / reindeer

Not done yet (phase 3-4): the makers' servo models in place of the boxes, printed parts joined and rounded, holes.
"""
import json
import math
import os
import time

import adsk.core
import adsk.fusion

import figlib as F

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GLOSS = ('#08090B', '#1BA05A', '#17542E')


def app():
    return adsk.core.Application.get()


def design():
    return adsk.fusion.Design.cast(app().activeProduct)


def spec():
    return json.load(open(os.path.join(REPO, 'spec', 'robot.json'), encoding='utf-8'))


def finish(g):
    return 'gloss' if g['color'] in GLOSS else 'metal' if g['color'] in ('#9EA3A8',) else 'matte'


def new_document():
    app().documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design().designType = adsk.fusion.DesignTypes.DirectDesignType
    return f'new document, orientation {app().preferences.generalPreferences.defaultModelingOrientation} (1 = Z up)'


def occurrences():
    root = design().rootComponent
    return {root.occurrences.item(i).component.name: root.occurrences.item(i) for i in range(root.occurrences.count)}


def link(name):
    t = time.time()
    s = spec()
    d = design()
    root = d.rootComponent
    for o in list(root.occurrences):
        if o.component.name == name:
            o.deleteMe()
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    b = F.Builder(comp)
    geoms = s['links'][name]['geoms']
    problems = []
    for g in geoms:
        if g['role'] == 'cut':
            continue
        body = b.build(g)
        body.name = g['name']
        body.appearance = F.appearance(d, g['color'], finish(g))
    for g in geoms:
        if g['role'] == 'cut':
            targets = [comp.bRepBodies.itemByName(n) for n in g['targets'] if comp.bRepBodies.itemByName(n)]
            if not targets:
                continue
            tool = b.build(g)
            try:
                b.cut(tool, targets, g['name'])
            except RuntimeError as e:
                problems.append(f"{g['name']}: {e}")
    if name == s['root']:
        occ.isGrounded = True
    else:
        occ.isGroundToParent = False
    return f'{name}: {comp.bRepBodies.count} bodies, {time.time() - t:.1f} s' + (f'; PROBLEMS {problems}' if problems else '')


def parametric():
    d = design()
    d.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return f'parametric, timeline {d.timeline.count}'


def joints():
    t = time.time()
    s = spec()
    root = design().rootComponent
    occ = occurrences()
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'joint points'
    sk.isVisible = False
    dirs = {'x': adsk.fusion.JointDirections.XAxisJointDirection, 'y': adsk.fusion.JointDirections.YAxisJointDirection,
            'z': adsk.fusion.JointDirections.ZAxisJointDirection}
    for j in s['joints']:
        sp = sk.sketchPoints.add(sk.modelToSketchSpace(F.P(j['origin'])))
        ji = root.asBuiltJoints.createInput(occ[j['child']], occ[j['parent']], adsk.fusion.JointGeometry.createByPoint(sp))
        ji.setAsRevoluteJointMotion(dirs[j['axis']])
        jt = root.asBuiltJoints.add(ji)
        jt.name = j['name']
        lim = adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion).rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = j['lower']
        lim.isMaximumValueEnabled = True
        lim.maximumValue = j['upper']
    return f"{len(s['joints'])} joints, {time.time() - t:.1f} s"


def pose(q):
    root = design().rootComponent
    for i in range(root.asBuiltJoints.count):
        jt = root.asBuiltJoints.item(i)
        adsk.fusion.RevoluteJointMotion.cast(jt.jointMotion).rotationValue = float(q.get(jt.name, 0.0))
    return f'posed {len(q)} joints'


def status():
    s = spec()
    d = design()
    occ = occurrences()
    problems, rows = [], []
    for name, l in s['links'].items():
        o = occ.get(name)
        if not o:
            problems.append(f'missing {name}')
            continue
        want = len([g for g in l['geoms'] if g['role'] != 'cut'])
        nb = o.component.bRepBodies.count
        if nb != want:
            problems.append(f'{name}: {nb} bodies, spec has {want} (a cut that splits a body adds one)')
        rows.append(f'{name}:{nb}')
    nj, count = 0, 0
    if d.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        tl = d.timeline
        count = tl.count
        for i in range(tl.count):
            it = tl.item(i)
            if not it.isGroup and it.healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState:
                problems.append(f'unhealthy: {it.name}')
        nj = d.rootComponent.asBuiltJoints.count
    return {'links': rows, 'joints': nj, 'timeline': count, 'problems': problems[:30]}


VIEWS = {'front': (0, 0), 'side': (90, 0), 'r34': (-55, 6), '34': (60, 8), 'back': (180, 0), 'top': (0, 89)}


def render(path, view='r34', w=900, h=900, height_cm=28.0, target_z_cm=12.0):
    az, el = (math.radians(v) for v in VIEWS[view])
    vp = app().activeViewport
    vp.visualStyle = adsk.core.VisualStyles.ShadedVisualStyle
    cam = vp.camera
    cam.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    cam.isSmoothTransition = False
    dvec = (math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el))
    cam.target = adsk.core.Point3D.create(0, 0, target_z_cm)
    cam.eye = adsk.core.Point3D.create(300 * dvec[0], 300 * dvec[1], target_z_cm + 300 * dvec[2])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1) if view != 'top' else adsk.core.Vector3D.create(1, 0, 0)
    cam.viewExtents = height_cm
    vp.camera = cam
    vp.refresh()
    root = design().rootComponent
    root.isJointsFolderLightBulbOn = False
    root.isSketchFolderLightBulbOn = False
    root.isConstructionFolderLightBulbOn = False
    if os.path.exists(path):
        os.remove(path)
    ok = vp.saveAsImageFile(path, w, h)
    return f'{view}: {path} ({ok}, written: {os.path.exists(path)})'


def interference():
    """Fusion's own clash check over every body; [(a, b, cm3)], largest first."""
    d = design()
    root = d.rootComponent
    bodies = adsk.core.ObjectCollection.create()
    for o in root.allOccurrences:
        for i in range(o.bRepBodies.count):
            bodies.add(o.bRepBodies.item(i))
    ii = d.createInterferenceInput(bodies)
    ii.areCoincidentFacesIncluded = False
    res = d.analyzeInterference(ii)
    if res is None:
        return None
    out = []
    for r in res:
        out.append((f'{r.entityOne.parentComponent.name}/{r.entityOne.name}', f'{r.entityTwo.parentComponent.name}/{r.entityTwo.name}',
                    round(r.interferenceBody.volume, 4)))
    return sorted(out, key=lambda x: -x[2])


def save(name='reindeer', project='my-projects', folder='reindeer'):
    a = app()
    proj = next(p for p in a.data.activeHub.dataProjects if p.name == project)
    root = proj.rootFolder
    f = next((root.dataFolders.item(i) for i in range(root.dataFolders.count) if root.dataFolders.item(i).name == folder), None)
    f = f or root.dataFolders.add(folder)
    doc = a.activeDocument
    if doc.isSaved and doc.dataFile:
        doc.save('rebuilt from spec/robot.json')
    else:
        doc.saveAs(name, f, 'reindeer: generated from spec/robot.json', '')
    return f'saved {doc.name}: isSaved {doc.isSaved}'
