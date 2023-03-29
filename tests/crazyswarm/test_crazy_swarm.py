# Copyright (C) 2021 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
import pytest
import logging
import os
import random
import rospy
import sys
import time

try:
    crazyswarm_path = os.environ['CRAZYSWARM_PATH']
    sys.path.append(
        os.path.join(crazyswarm_path, 'ros_ws/src/crazyswarm/scripts')
    )
    sys.path.append(os.path.join(crazyswarm_path, 'ros_ws/src/crazyflie_ros'))
    from pycrazyswarm import Crazyswarm
    from crazyswarm.msg import GenericLogData
except KeyError:
    print('CRAZYSWARM_PATH or CRAZYSWARM_YAML not set', file=sys.stderr)
    # pytest.skip()
    sys.exit(17)
except ImportError:
    print('Failed to import pycrazyswarm', file=sys.stderr)
    # pytest.skip()
    sys.exit(18)

logger = logging.getLogger(__name__)

yaml = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '../../swarms',
    os.environ['CRAZYSWARM_YAML']
)
swarm = Crazyswarm(crazyflies_yaml=yaml)


class TestCrazyswarm:
    def test_get_cf_param(self):
        '''Test that we can read parameters from all crazyflies in swarm'''
        for cf in swarm.allcfs.crazyflies:
            assert cf.getParam('stabilizer/estimator') == 1

    def test_set_cf_param(self):
        '''Test that we can set a param on a crazyflie and read it back'''
        index = random.randrange(0, len(swarm.allcfs.crazyflies) - 1)
        cf = swarm.allcfs.crazyflies[index]
        cf.setParam('commander/enHighLevel', 0)

        for i, cf in enumerate(swarm.allcfs.crazyflies):
            if i == index:
                assert cf.getParam('commander/enHighLevel') == 0
            else:
                assert cf.getParam('commander/enHighLevel') == 1
        swarm.allcfs.setParam('commander/enHighLevel', 1)

    def test_set_cf_param_broadcast(self):
        log_val = 0
        def log_callback(value):
            nonlocal log_val
            log_val = int(value.values[0])

        def broadcast_and_validate(value: int) -> bool:
            # The delay from a broadcast is sent until it shows up in the log topic. Not sure why it takes so long
            topic_max_wait_time = 10.0

            # Broadcast a value
            swarm.allcfs.setParam('system/testLogParam', value)

            # Wait for the value to appear in the log
            end_time = time.time() + topic_max_wait_time
            while time.time() < end_time:
                rospy.sleep(0.3)
                if log_val == value:
                    return True
            return False

        # Set up logging for the first CF
        rospy.Subscriber('/cf' + str(swarm.allcfs.crazyflies[0].id) + '/log1', GenericLogData, log_callback, queue_size=1)

        assert broadcast_and_validate(47)
        assert broadcast_and_validate(11)
