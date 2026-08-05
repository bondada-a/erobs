"""Runnable check for the GoalStatus->mbbo index mapping. No bridge/IOC needed.
    python3 src/erob_epics/test/test_mapping.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from action_msgs.msg import GoalStatus  # noqa: E402
from action_status_to_pv import index_for, IDLE  # noqa: E402


def _g(status, sec):
    return SimpleNamespace(
        status=status,
        goal_info=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=0)),
    )


def test():
    assert index_for([]) == IDLE  # no goal -> Idle(0)
    assert index_for([_g(GoalStatus.STATUS_EXECUTING, 1)]) == 1  # Running
    assert index_for([_g(GoalStatus.STATUS_SUCCEEDED, 1)]) == 2  # Succeeded
    assert index_for([_g(GoalStatus.STATUS_ABORTED, 1)]) == 3  # Failed
    assert index_for([_g(GoalStatus.STATUS_CANCELED, 1)]) == 4  # Canceled
    # newest goal wins over an older terminal one
    assert (
        index_for(
            [_g(GoalStatus.STATUS_SUCCEEDED, 1), _g(GoalStatus.STATUS_EXECUTING, 2)]
        )
        == 1
    )
    print("ok")


if __name__ == "__main__":
    test()
