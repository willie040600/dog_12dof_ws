"""Lockstep physics like Isaac Lab: N fixed physics steps, then one app update for rendering/OmniGraph.

Kit's own physics stepping is switched off (/app/player/playSimulations), so the policy controller sees
every physics step exactly as during training. Import only after SimulationApp has started.
"""
import carb.settings
import omni.physx
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager


class Lockstep:
    def __init__(self, app, physics_dt, decimation):
        self._app = app
        self.physics_dt = physics_dt
        self.decimation = decimation
        settings = carb.settings.get_settings()
        settings.set("/persistent/simulation/minFrameRate", int(round(1.0 / physics_dt)) // decimation)
        settings.set("/app/player/playSimulations", False)
        omni.timeline.get_timeline_interface().play()
        app.update()
        self._physx = omni.physx.get_physx_simulation_interface()
        self._steps = 0

    def step(self, pre_physics):
        """pre_physics() then one physics step, decimation times; then render and tick OmniGraph once."""
        for _ in range(self.decimation):
            pre_physics()
            self._physx.simulate(self.physics_dt, self._steps * self.physics_dt)
            self._physx.fetch_results()
            self._steps += 1
        self._app.update()

    @property
    def sim_time(self):
        return SimulationManager.get_simulation_time()

    @property
    def physics_steps(self):
        return SimulationManager.get_num_physics_steps()
