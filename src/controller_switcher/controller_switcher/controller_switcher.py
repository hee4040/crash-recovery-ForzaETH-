#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal
import time
import atexit


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__("controller_switcher")

        # 종료 시 자동 정리하도록 등록
        atexit.register(self.cleanup_processes)

        self.proc = None                    # 현재 실행 중인 launch 프로세스
        self.current_mode = None            # 현재 동작 모드(PP/TLN)
        self.last_ignored_state = None      # 반복된 Unknown state 로그 방지
        self.last_same_mode_log = None      # 동일한 모드 반복 로그 방지

        # /state_machine 에서 모드 전환 상태 받기
        self.create_subscription(
            String,
            "/state",
            self.mode_cb,
            10
        )

        self.get_logger().info("controller_switcher ready.")

    #  -------------- 프로세스 종료 관련 -----------------
    def kill_current_process(self):
        # 현재 실행 중인 main launch 프로세스를 종료
        if self.proc is None:
            return

        try:
            pgid = os.getpgid(self.proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(0.3)
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass

        # 혹시 남아있는 tln_inference 개별 프로세스도 제거
        self.kill_by_names(["tln_inference"])

        self.proc = None

    def kill_by_names(self, patterns):
        # 이름 패턴으로 프로세스를 강제 종료
        for pat in patterns:
            try:
                pids = subprocess.check_output(
                    f"pgrep -f '{pat}'", shell=True
                ).decode().split()

                for pid in pids:
                    try:
                        os.kill(int(pid), signal.SIGKILL)
                        self.get_logger().warn(f"Killed [{pat}] PID={pid}")
                    except ProcessLookupError:
                        pass

            except subprocess.CalledProcessError:
                pass  # 해당 패턴의 프로세스 없음

    def cleanup_processes(self):
        # 노드 종료 시 모든 관련 프로세스를 정리
        self.get_logger().warn("ControllerSwitcher shutting down → cleaning all processes")

        # 현재 실행 중인 프로세스 종료
        if self.proc is not None:
            try:
                pgid = os.getpgid(self.proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass

        # name 기반으로 잔여 프로세스도 정리
        kill_list = [
            "tln_inference",
            "controller_manager_cpp",
            "ros2 launch controller_cpp",
            "ros2 launch ofc"
        ]

        for name in kill_list:
            os.system(f"pkill -9 -f '{name}'")

    # ----------------- 프로세스 시작 ---------------------
    def start_process(self, cmd):
        """새로운 ros2 launch 프로세스를 백그라운드로 실행"""
        self.get_logger().info(f"Starting: {cmd}")
        self.proc = subprocess.Popen(
            cmd,
            shell=True,
            preexec_fn=os.setsid  # 자식 프로세스를 별도 그룹으로 실행
        )

    # ------------------ 상태 콜백 --------------------
    def mode_cb(self, msg):
        state = msg.data.upper()

        # 1) NORMAL → PP, RECOVERY → TLN
        if "NORMAL" in state:
            mode = "PP"
        elif "RECOVERY" in state:
            mode = "TLN"
        else:
            # 같은 Unknown state가 반복되면 로그 생략
            if self.last_ignored_state != state:
                self.get_logger().error(f"Unknown state '{state}', ignoring.")
                self.last_ignored_state = state
            return

        # 2) 이미 동일한 모드면 무시
        if mode == self.current_mode:
            if self.last_same_mode_log != mode:
                self.get_logger().info(f"Already in mode {mode}, ignoring.")
                self.last_same_mode_log = mode
            return
        else:
            self.last_same_mode_log = None

        # 3) 실제 모드 전환
        self.get_logger().warn(f"Controller mode change: {self.current_mode} → {mode}")

        self.kill_current_process()

        if mode == "PP":
            # TLN 잔여 프로세스가 남아있으면 반드시 제거
            self.kill_by_names(["tln_inference"])
            self.start_process("ros2 launch controller_cpp controller_launch.xml mode:=PP")

        elif mode == "TLN":
            self.start_process("ros2 launch ofc ofc_enabled_tln.launch.py")

        self.current_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcher()

    try:
        rclpy.spin(node)
    finally:
        node.cleanup_processes()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
