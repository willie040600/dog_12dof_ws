"""OmniGraph ROS 2 publishers/subscribers. Import only after SimulationApp has started."""
import omni.graph.core as og
from pxr import Sdf

# OmniGraph node types keep the isaacsim.ros2.bridge prefix in 6.x (extension isaacsim.ros2.nodes)
ROS = 'isaacsim.ros2.bridge.'
TICK = ('Tick', 'omni.graph.action.OnPlaybackTick')
SIM_TIME = ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime')
CONTEXT = ('Context', ROS + 'ROS2Context')
RENDER_PRODUCT = ('RenderProduct', 'isaacsim.core.nodes.IsaacCreateRenderProduct')


def graph(stage, path, nodes, values, connections):
    try:
        og.Controller.edit({'graph_path': path, 'evaluator_name': 'execution'}, {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.SET_VALUES: values,
            og.Controller.Keys.CONNECT: connections,
        })
    except Exception:
        stage.RemovePrim(path)  # do not leave half-built graphs behind
        raise


def clock_graph(stage, topic='/clock'):
    graph(stage, '/ROS/clock',
          [TICK, SIM_TIME, CONTEXT, ('PubClock', ROS + 'ROS2PublishClock')],
          [('SimTime.inputs:resetOnStop', True), ('PubClock.inputs:topicName', topic)],
          [('Tick.outputs:tick', 'PubClock.inputs:execIn'),
           ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),
           ('Context.outputs:context', 'PubClock.inputs:context')])


def joint_state_graph(stage, art_path, topic):
    fields = ('jointNames', 'jointPositions', 'jointVelocities', 'jointEfforts', 'jointDofTypes',
              'stageMetersPerUnit', 'sensorTime')
    graph(stage, '/ROS/joint_states',
          [TICK, SIM_TIME, CONTEXT,
           ('ReadJointState', 'isaacsim.sensors.physics.IsaacReadJointState'),
           ('PubJointState', ROS + 'ROS2PublishJointState')],
          [('SimTime.inputs:resetOnStop', True),
           ('ReadJointState.inputs:prim', [Sdf.Path(art_path)]),
           ('PubJointState.inputs:topicName', topic)],
          [('Tick.outputs:tick', 'ReadJointState.inputs:execIn'),
           ('ReadJointState.outputs:execOut', 'PubJointState.inputs:execIn'),
           ('SimTime.outputs:simulationTime', 'PubJointState.inputs:timeStamp'),
           ('Context.outputs:context', 'PubJointState.inputs:context')]
          + [('ReadJointState.outputs:%s' % f, 'PubJointState.inputs:%s' % f) for f in fields])


def joint_command_graph(stage, art_path, topic):
    """sensor_msgs/JointState commands (from ros2_control) -> articulation controller."""
    graph(stage, '/ROS/joint_commands',
          [TICK, CONTEXT,
           ('SubJointCmd', ROS + 'ROS2SubscribeJointState'),
           ('Articulation', 'isaacsim.core.nodes.IsaacArticulationController')],
          [('SubJointCmd.inputs:topicName', topic),
           ('Articulation.inputs:targetPrim', [Sdf.Path(art_path)])],
          [('Tick.outputs:tick', 'SubJointCmd.inputs:execIn'),
           ('Tick.outputs:tick', 'Articulation.inputs:execIn'),
           ('Context.outputs:context', 'SubJointCmd.inputs:context')]
          + [('SubJointCmd.outputs:%s' % a, 'Articulation.inputs:%s' % a)
             for a in ('jointNames', 'positionCommand', 'velocityCommand', 'effortCommand')])


def imu_graph(stage, imu_path, topic='/imu', frame_id='imu_link'):
    graph(stage, '/ROS/imu',
          [TICK, SIM_TIME, CONTEXT, ('ReadIMU', 'isaacsim.sensors.physics.IsaacReadIMU'),
           ('PubImu', ROS + 'ROS2PublishImu')],
          [('ReadIMU.inputs:imuPrim', [Sdf.Path(imu_path)]), ('ReadIMU.inputs:readGravity', True),
           ('PubImu.inputs:frameId', frame_id), ('PubImu.inputs:topicName', topic)],
          [('Tick.outputs:tick', 'ReadIMU.inputs:execIn'),
           ('ReadIMU.outputs:execOut', 'PubImu.inputs:execIn'),
           ('ReadIMU.outputs:angVel', 'PubImu.inputs:angularVelocity'),
           ('ReadIMU.outputs:linAcc', 'PubImu.inputs:linearAcceleration'),
           ('ReadIMU.outputs:orientation', 'PubImu.inputs:orientation'),
           ('SimTime.outputs:simulationTime', 'PubImu.inputs:timeStamp'),
           ('Context.outputs:context', 'PubImu.inputs:context')])


def camera_graph(stage, cam_path, size, topic='/camera/image_raw', info_topic='/camera/camera_info',
                 frame_id='camera_optical_link'):
    width, height = size
    graph(stage, '/ROS/camera',
          [TICK, CONTEXT, RENDER_PRODUCT,
           ('PubImage', ROS + 'ROS2CameraHelper'),
           ('PubInfo', ROS + 'ROS2CameraInfoHelper')],
          [('RenderProduct.inputs:cameraPrim', [Sdf.Path(cam_path)]),
           ('RenderProduct.inputs:width', width), ('RenderProduct.inputs:height', height),
           ('PubImage.inputs:type', 'rgb'), ('PubImage.inputs:topicName', topic),
           ('PubImage.inputs:frameId', frame_id),
           ('PubInfo.inputs:topicName', info_topic),
           ('PubInfo.inputs:frameId', frame_id)],
          [('Tick.outputs:tick', 'RenderProduct.inputs:execIn'),
           ('RenderProduct.outputs:execOut', 'PubImage.inputs:execIn'),
           ('RenderProduct.outputs:execOut', 'PubInfo.inputs:execIn'),
           ('RenderProduct.outputs:renderProductPath', 'PubImage.inputs:renderProductPath'),
           ('RenderProduct.outputs:renderProductPath', 'PubInfo.inputs:renderProductPath'),
           ('Context.outputs:context', 'PubImage.inputs:context'),
           ('Context.outputs:context', 'PubInfo.inputs:context')])


def lidar_graph(stage, lidar_path, kind, topic, frame_id='laser_link'):
    graph(stage, '/ROS/lidar',
          [TICK, CONTEXT, RENDER_PRODUCT, ('PubScan', ROS + 'ROS2RtxLidarHelper')],
          [('RenderProduct.inputs:cameraPrim', [Sdf.Path(lidar_path)]),
           ('PubScan.inputs:type', kind), ('PubScan.inputs:topicName', topic),
           ('PubScan.inputs:frameId', frame_id)],
          [('Tick.outputs:tick', 'RenderProduct.inputs:execIn'),
           ('RenderProduct.outputs:execOut', 'PubScan.inputs:execIn'),
           ('RenderProduct.outputs:renderProductPath', 'PubScan.inputs:renderProductPath'),
           ('Context.outputs:context', 'PubScan.inputs:context')])
