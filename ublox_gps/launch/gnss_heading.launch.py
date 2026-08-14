""" GNSS Heading (F9H) launch file - production. """
import os

import ament_index_python.packages
import launch
import launch_ros.actions

# Only the DATA topics are remapped under TOPIC_NS; lifecycle/parameter infrastructure
# (transition_event, change_state, get/set parameters, ...) deliberately stays
# under the node name, e.g. /automatepro_gnss_heading_node/transition_event.
TOPIC_NS = '/sensor/gnss/heading'

# Driver data outputs advertised with a private (~) name; remapping drops the
# node-name segment and places them under TOPIC_NS.
_PRIVATE_TOPICS = ('fix', 'fix_velocity', 'navpvt')

# Driver data outputs advertised with a relative name; at root they would sit
# at '/<name>', so each is remapped under TOPIC_NS. 'navheading' is handled
# separately below (published as 'true_heading').
_RELATIVE_TOPICS = (
    'navrelposned', 'navstatus', 'navposecef', 'navposllh', 'navcov',
    'navclock', 'navsol', 'navvelned', 'navsvin', 'navsvinfo', 'navinfo',
    'navstate', 'navatt', 'monhw', 'monsys', 'nmea', 'aidalm', 'aideph',
    'aidhui', 'rxmalm', 'rxmeph', 'rxmraw', 'rxmrtcm', 'rxmsfrb', 'esfins',
    'esfmeas', 'esfraw', 'esfstatus', 'hnrpvt', 'timtm2', 'imu_meas',
    'interrupt_time', 'raw_data_stream',
)

_REMAPPINGS = (
    [(f'~/{t}', f'{TOPIC_NS}/{t}') for t in _PRIVATE_TOPICS]
    + [('navheading', f'{TOPIC_NS}/true_heading')]
    + [(t, f'{TOPIC_NS}/{t}') for t in _RELATIVE_TOPICS]
    # The F9H rover derives heading from the moving baseline to the F9P and takes
    # its corrections from the F9P over UART2. Feeding it fixed-network NTRIP RTCM
    # on /rtcm makes it attempt an absolute RTK solve instead, which invalidates
    # NAV-RELPOSNED (heading reads 0). Isolate its /rtcm to an unused topic.
    + [('/rtcm', f'{TOPIC_NS}/rtcm_unused')]
)


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('ublox_gps'),
        'config')
    params = os.path.join(config_directory, 'gnss_heading_params.yaml')
    ublox_gps_node = launch_ros.actions.Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='automatepro_gnss_heading_node',
        output='both',
        parameters=[params],
        remappings=_REMAPPINGS)

    return launch.LaunchDescription([ublox_gps_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=ublox_gps_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
