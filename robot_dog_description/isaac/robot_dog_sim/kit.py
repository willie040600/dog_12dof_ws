"""Kit extensions and stage set-up. Import only after SimulationApp has started."""
import omni.kit.app
import omni.usd

EXTENSIONS = ('isaacsim.asset.importer.urdf', 'isaacsim.core.nodes', 'isaacsim.sensors.physics',
              'isaacsim.sensors.experimental.rtx', 'isaacsim.ros2.bridge', 'isaacsim.ros2.nodes')


def enable_extensions(names=EXTENSIONS):
    ext = omni.kit.app.get_app().get_extension_manager()
    for name in names:
        ext.set_extension_enabled_immediate(name, True)  # no-op if already on or absent


def new_stage(app):
    omni.usd.get_context().new_stage()
    app.update()
    return omni.usd.get_context().get_stage()
