import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('oak_d_camera'),
                                'rviz', 'rgbd_stereo_pcl.rviz')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')


    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D-LITE')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    cam_pos_x  = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y  = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z  = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll   = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch  = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw    = LaunchConfiguration('cam_yaw',       default = '0.0')

    lrcheck         = LaunchConfiguration('lrcheck',        default = True)
    extended        = LaunchConfiguration('extended',       default = False)
    subpixel        = LaunchConfiguration('subpixel',       default = True)
    confidence      = LaunchConfiguration('confidence',     default = 200)
    LRchecktresh    = LaunchConfiguration('LRchecktresh',   default = 5)
    use_rviz        = LaunchConfiguration('use_rviz',       default = True)
    use_depth       = LaunchConfiguration('use_depth',      default = True)
    use_disparity   = LaunchConfiguration('use_disparity',  default = True)
    use_lr_raw      = LaunchConfiguration('use_lr_raw',     default = True)
    use_pointcloud  = LaunchConfiguration('use_pointcloud', default = True)
    pc_color        = LaunchConfiguration('pc_color',       default = True)
    only_rgb        = LaunchConfiguration('only_rgb',       default = False)


    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_lrcheck_cmd = DeclareLaunchArgument(
        'lrcheck',
        default_value=lrcheck,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_extended_cmd = DeclareLaunchArgument(
        'extended',
        default_value=extended,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_subpixel_cmd = DeclareLaunchArgument(
        'subpixel',
        default_value=subpixel,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_confidence_cmd = DeclareLaunchArgument(
        'confidence',
        default_value=confidence,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_LRchecktresh_cmd = DeclareLaunchArgument(
        'LRchecktresh',
        default_value=LRchecktresh,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz,
        description='When True create a RVIZ window.')

    declare_use_depth_cmd = DeclareLaunchArgument(
        'use_depth',
        default_value=use_depth,
        description='Use depth image.')

    declare_use_disparity_cmd = DeclareLaunchArgument(
        'use_disparity',
        default_value=use_disparity,
        description='Use disparity image.')

    declare_use_lr_raw_cmd = DeclareLaunchArgument(
        'use_lr_raw',
        default_value=use_lr_raw,
        description='Use left and right raw images.')

    declare_use_pointcloud_cmd = DeclareLaunchArgument(
        'use_pointcloud',
        default_value=use_pointcloud,
        description='Use point cloud.')

    declare_point_cloud_color_cmd = DeclareLaunchArgument(
        'pc_color',
        default_value=pc_color,
        description='Use color in the point cloud.')

    declare_only_rgb_cmd = DeclareLaunchArgument(
        'only_rgb',
        default_value=only_rgb,
        description='Use only RGB image.')

    rgbd_stereo_node = launch_ros.actions.Node(
            package='oak_d_camera', executable='rgbd_stereo_node',
            output='screen',
            namespace='c1',
            parameters=[{'tf_prefix': 'oak1'},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'use_depth': use_depth},
                        {'use_disparity': use_disparity},
                        {'use_lr_raw': use_lr_raw},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color},
                        {'only_rgb': only_rgb},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color}])

    rgbd_stereo_node2 = launch_ros.actions.Node(
            package='oak_d_camera', executable='rgbd_stereo_node',
            output='screen',
            namespace='c2',
            parameters=[{'tf_prefix': 'oak2'},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'use_depth': use_depth},
                        {'use_disparity': use_disparity},
                        {'use_lr_raw': use_lr_raw},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color},
                        {'only_rgb': only_rgb},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color}])

    rgbd_stereo_node3 = launch_ros.actions.Node(
            package='oak_d_camera', executable='rgbd_stereo_node',
            output='screen',
            namespace='c3',
            parameters=[{'tf_prefix': 'oak3'},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'use_depth': use_depth},
                        {'use_disparity': use_disparity},
                        {'use_lr_raw': use_lr_raw},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color},
                        {'only_rgb': only_rgb},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color}])

    rgbd_stereo_node4 = launch_ros.actions.Node(
            package='oak_d_camera', executable='rgbd_stereo_node',
            output='screen',
            namespace='c4',
            parameters=[{'tf_prefix': 'oak4'},
                        {'lrcheck': lrcheck},
                        {'extended': extended},
                        {'subpixel': subpixel},
                        {'confidence': confidence},
                        {'LRchecktresh': LRchecktresh},
                        {'use_depth': use_depth},
                        {'use_disparity': use_disparity},
                        {'use_lr_raw': use_lr_raw},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color},
                        {'only_rgb': only_rgb},
                        {'use_pointcloud': use_pointcloud},
                        {'pc_color': pc_color}])


    #delayed_rgbd_stereo_node2 = RegisterEventHandler(OnProcessStart(target_action=rgbd_stereo_node,on_start=[LogInfo(msg='Turtlesim started, spawning turtle'),rgbd_stereo_node2]))
    delayed_rgbd_stereo_node2 = TimerAction(period=3.0, actions=[rgbd_stereo_node2])
    delayed_rgbd_stereo_node3 = TimerAction(period=6.0, actions=[rgbd_stereo_node3])
    delayed_rgbd_stereo_node4 = TimerAction(period=9.0, actions=[rgbd_stereo_node4])

    rviz_node = launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz],
            condition=IfCondition(use_rviz))


    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_camera_model_cmd)

    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)

    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(declare_lrcheck_cmd)
    ld.add_action(declare_extended_cmd)
    ld.add_action(declare_subpixel_cmd)
    ld.add_action(declare_confidence_cmd)
    ld.add_action(declare_LRchecktresh_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_depth_cmd)
    ld.add_action(declare_use_disparity_cmd)
    ld.add_action(declare_use_lr_raw_cmd)
    ld.add_action(declare_use_pointcloud_cmd)
    ld.add_action(declare_point_cloud_color_cmd)
    ld.add_action(declare_only_rgb_cmd)

    ld.add_action(rgbd_stereo_node)
    ld.add_action(delayed_rgbd_stereo_node2)
    ld.add_action(delayed_rgbd_stereo_node3)
    ld.add_action(delayed_rgbd_stereo_node4)
    ld.add_action(rviz_node)

    return ld
