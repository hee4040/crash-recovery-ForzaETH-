from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
 #---------프로젝트용 코드-------------       
# from state_machine.state_types import StateType

# def timetrials_transition(state_machine: StateMachine) -> StateType:
#     """
#     (임시 버전)
#     - PF stable은 아직 안 씀
#     - 오직 collision_detected 만으로 NORMAL <-> RECOVERY 전환

#         collision_detected == False  -> NORMAL
#         collision_detected == True   -> RECOVERY
#     """
#     # 없으면 False로 간주 (초기 디폴트)
#     cd = getattr(state_machine, 'collision_detected', False)
#     pf_stable = getattr(state_machine, 'pf_stable', False)
#     backward_done = getattr(state_machine, 'backward_done', False)
#     cur = state_machine.state

#     # 혹시 이전에 GB_TRACK 같은 상태로 시작했으면 NORMAL로 정리
#     if cur not in (StateType.NORMAL, StateType.RECOVERY):
#         state_machine.get_logger().warn(
#             f"[FSM] INVALID initial state {cur} → NORMAL로 강제 전환"
#         )
#         state_machine.state = StateType.NORMAL
#         cur = StateType.NORMAL

#     if cur == StateType.NORMAL:
#         if cd:
#             return StateType.RECOVERY
#         else:
#             return StateType.NORMAL

#     if cur == StateType.RECOVERY:
#         if pf_stable:
#             state_machine.get_logger().info(
#             "[FSM] PF stable 신호 감지 → NORMAL 복귀"
#             )
#             #state_machine.collision_detected = False
#             state_machine.pf_stable = False 
#             return StateType.NORMAL
#         # if backward_done and pf_stable:
#         #     # 복귀 후 backward_done을 false로 reset
#         #     state_machine.backward_done = False
#         #     return StateType.NORMAL
#         # if not cd:
#         #     return StateType.NORMAL
#         else:
#             return StateType.RECOVERY

def timetrials_transition(sm):
    cd = sm.collision_detected
    pf_stable = sm.pf_stable
    backward_done = sm.backward_done
    cur = sm.state

    # NORMAL 상태
    if cur == StateType.NORMAL:
        if cd:
            sm.get_logger().warn("[FSM] Collision detected → RECOVERY")
            return StateType.RECOVERY
        return StateType.NORMAL

    # RECOVERY 상태
    if cur == StateType.RECOVERY:

        # 후진 안 끝났으면 PF stable 무시
        if not backward_done:
            return StateType.RECOVERY

        # 후진 끝난 다음은 PF stable 체크
        if backward_done and pf_stable:
            sm.get_logger().info("[FSM] backward_done & PF stable → NORMAL")

            # 정상 퇴출 처리
            sm.collision_detected = False
            sm.backward_done = False
            sm.pf_stable = False
            return StateType.NORMAL
        
        return StateType.RECOVERY

    return StateType.NORMAL


#--------------프로젝트용 코드----------------
def head_to_head_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)

        case StateType.TRAILING_TO_GBTRACK:
            return SpliniTrailingToGbtrackTransition(state_machine)

        case StateType.OVERTAKE:
            return SpliniOvertakingTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY

def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            return StateType.TRAILING

        # 아래와 같이 바로 GB_TRACK로 전환하지 않고 TRAILING_TO_GBTRACK로 전환하도록 수정
        elif gb_free and state_machine._check_close_to_raceline:
            return StateType.TRAILING_TO_GBTRACK


        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY

def SpliniTrailingToGbtrackTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING_TO_GBTRACK`"""
    # GB_TRACK 이외의 다른 상태로 return 할 때에는 trailing_to_gbtrack_count를 0으로 리셋해주기
    
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.TRAILING


        elif gb_free and state_machine._check_close_to_raceline:

            state_machine.trailing_to_gbtrack_count += 1

            # gb_free의 횟수가 threshold를 넘기면 그때는 진짜로 gbtrack으로 전환
            if state_machine.trailing_to_gbtrack_count >= state_machine.trailing_to_gbtrack_counting_threshold:
                state_machine.trailing_to_gbtrack_count = 0
                return StateType.GB_TRACK

            else:
                return StateType.TRAILING_TO_GBTRACK

        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.OVERTAKE
        else:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.TRAILING
    else:
        state_machine.trailing_to_gbtrack_count = 0
        return StateType.FTGONLY



def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_sector
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY
        