from __future__ import annotations

from typing import List, TYPE_CHECKING
from f110_msgs.msg import Wpnt
from state_machine.state_types import StateType
if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine

def DefaultStateLogic(state_machine: StateMachine) -> List[Wpnt]:
    """
    This is a global state that incorporates the other states
    """
    match state_machine.state:
        # === 프로젝트용 추가 상태 ===
        case StateType.NORMAL:
            # NORMAL 은 GB_TRACK 과 동일하게 글로벌 레이싱 라인 따라감
            return GlobalTracking(state_machine)

        case StateType.RECOVERY:
            # RECOVERY 모드: TLN 이 직접 cmd_vel 을 내보내므로 웨이포인트는 비움
            return RecoveryMode(state_machine)

        # === 기존 상태들 ===
        case StateType.GB_TRACK:
            return GlobalTracking(state_machine)
        case StateType.TRAILING:
            return Trailing(state_machine)
        case StateType.OVERTAKE:
            return Overtaking(state_machine)
        case StateType.FTGONLY:
            return FTGOnly(state_machine)
        case StateType.TRAILING_TO_GBTRACK:
            return Trailing_to_gbtrack(state_machine)
        case _:
            raise NotImplementedError(f"State {state_machine.state} not recognized")


"""
Here we define the behaviour in the different states.
Every function should be fairly concise, and output an array of f110_msgs.Wpnt
"""
def GlobalTracking(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def RecoveryMode(state_machine: StateMachine) -> List[Wpnt]:
    """
    Recovery 모드일 때는 TLN이 직접 cmd_vel을 내보내므로
    여기서는 웨이포인트를 비워둔다.
    """
    return []
    

def Trailing_to_gbtrack(state_machine: StateMachine) -> List[Wpnt]:
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Trailing(state_machine: StateMachine) -> List[Wpnt]:
    # This allows us to trail on the last valid spline if necessary
    if state_machine.last_valid_avoidance_wpnts is not None:
        splini_wpts = state_machine.get_splini_wpts()
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]
    else:
        s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
        return [state_machine.glb_wpnts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def Overtaking(state_machine: StateMachine) -> List[Wpnt]:
    splini_wpts = state_machine.get_splini_wpts()
    s = int(state_machine.cur_s/state_machine.waypoints_dist + 0.5)
    return [splini_wpts[(s + i)%state_machine.num_glb_wpnts] for i in range(state_machine.params.n_loc_wpnts)]

def FTGOnly(state_machine: StateMachine) -> List[Wpnt]:
    """No waypoints are generated in this follow the gap only state, all the 
    control inputs are generated in the control node.
    """
    return []