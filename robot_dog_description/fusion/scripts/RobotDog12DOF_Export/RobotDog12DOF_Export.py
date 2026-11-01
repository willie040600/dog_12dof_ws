"""Export the RobotDog12DOF design: per-link STL + robot_dog_fusion.json.

Then run tools/generate_description.py to rebuild the ROS 2 package from them.
"""
import adsk.core
import importlib
import os
import sys
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
_BUILD = os.path.join(os.path.dirname(_HERE), 'RobotDog12DOF')


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        if _BUILD not in sys.path:
            sys.path.insert(0, _BUILD)
        import RobotDog12DOF
        importlib.reload(RobotDog12DOF)
        dlg = ui.createFolderDialog()
        dlg.title = 'Folder for meshes_raw/ and robot_dog_fusion.json'
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        data = RobotDog12DOF.export(dlg.folder)
        ui.messageBox('Exported %d links and %d joints to\n%s' % (len(data['links']), len(data['joints']), dlg.folder),
                      'RobotDog12DOF Export')
    except Exception:
        if ui:
            ui.messageBox('Failed:\n' + traceback.format_exc(), 'RobotDog12DOF Export')
