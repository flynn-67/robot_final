import math

from modules.base_bt_nodes import (
    BTNodeList,
    Status,
    Node,
    Sequence,
    Fallback,
    ReactiveSequence,
    ReactiveFallback,
)

from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics,
    ActionWithROSAction,
    ActionWithROSService,
)

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from turtlebot3_camera_service_interfaces.srv import ImageCapture


# =========================
# 0. 이 시나리오에서 사용할 BT 노드 이름 등록
# =========================

CUSTOM_ACTION_NODES = [
    "MoveToGoal",
    "CaptureImage",
    "ReturnToStart",
]

CUSTOM_CONDITION_NODES = [
    "WaitForGoal",
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


# =========================
# 1. /bt/goal_pose 기다리기 + 시작 위치 저장
# =========================

class WaitForGoal(ConditionWithROSTopics):
    """
    /bt/goal_pose, /<ns>/amcl_pose 를 구독해서:
      - goal_pose: 블랙보드['goal_pose'] 에 저장
      - start_pose: 블랙보드['start_pose'] 에 저장 (최초 한 번만)

    동작 규칙
    - 현재 goal 이 "실행 중"이면 계속 SUCCESS (시퀀스 계속 진행)
    - 아직 goal 이 없으면 FAILURE (대기)
    - 새 goal 이 오면: 한 번 goal_pose 설정 + 실행 상태로 전환하고 SUCCESS
    - ReturnToStart 에서 완료 표시된 goal 은 다시 사용하지 않음
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        topics = [
            (PoseStamped, "/bt/goal_pose", "goal"),
            (PoseWithCovarianceStamped,
             f"{ns}/amcl_pose" if ns else "/amcl_pose",
             "amcl"),
        ]
        super().__init__(name, agent, topics)

    def _predicate(self, agent, bb) -> bool:
        cache = self._cache

        # 1) 이미 어떤 goal 이 "실행 중"이면 계속 SUCCESS 유지
        if bb.get("goal_active", False) and bb.get("goal_pose") is not None:
            return True

        # 2) 아직 실행 중인 goal 이 없으면, 새 goal 이 들어왔는지 확인
        if "goal" not in cache:
            # 아직 goal 자체가 안 들어온 상태 → 대기 (FAILURE)
            return False

        goal: PoseStamped = cache["goal"]

        # stamp 를 이용해서 "이 goal 이 이전에 끝난 goal 인지" 구분
        cur_stamp = (goal.header.stamp.sec, goal.header.stamp.nanosec)
        completed_stamp = bb.get("completed_goal_stamp", None)

        # 2-1) 이미 한 번 끝낸 goal 이면 다시 사용하지 않음 → 대기
        if completed_stamp == cur_stamp:
            return False

        # 2-2) 여기까지 왔으면 "새 goal" 이라고 판단
        #      이번 tick 부터는 이 goal 을 실행 중인 것으로 취급
        bb["goal_pose"] = goal
        bb["goal_active"] = True

        # 시작 위치 저장 (최초 한 번만)
        if "start_pose" not in bb:
            if "amcl" in cache:
                amcl: PoseWithCovarianceStamped = cache["amcl"]
                sp = PoseStamped()
                sp.header = amcl.header
                sp.pose = amcl.pose.pose
                bb["start_pose"] = sp
            else:
                # amcl_pose를 못 받았으면 일단 map (0,0)에 시작점
                sp = PoseStamped()
                sp.header.frame_id = "map"
                sp.pose.orientation.w = 1.0
                bb["start_pose"] = sp

        # 새 goal 을 받았으니 이번 tick 은 SUCCESS
        return True



# =========================
# 2. Nav2로 goal 이동
# =========================

class MoveToGoal(ActionWithROSAction):
    """
    블랙보드['goal_pose'] 를 Nav2 NavigateToPose 액션으로 보내는 노드.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, bb):
        goal_pose: PoseStamped = bb.get("goal_pose")
        if goal_pose is None:
            # goal이 없으면 실패
            return None
        msg = NavigateToPose.Goal()
        msg.pose = goal_pose
        return msg

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb["nav_result"] = "succeeded"
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb["nav_result"] = "canceled"
            return Status.FAILURE
        else:
            bb["nav_result"] = f"failed_status_{status_code}"
            return Status.FAILURE


# =========================
# 3. 사진 찍기
# =========================

class CaptureImage(ActionWithROSService):
    """
    /image_capture 서비스를 호출해서 이미지를 저장하는 노드.
    filename 파라미터는 XML에서 인자로 전달.
    """

    def __init__(self, name, agent, filename="bt_capture.png"):
        ns = agent.ros_namespace or ""
        service_name = f"{ns}/image_capture" if ns else "/image_capture"
        super().__init__(name, agent, (ImageCapture, service_name))
        self.filename = filename

    def _build_request(self, agent, bb):
        req = ImageCapture.Request()
        req.filename = self.filename
        return req

    def _interpret_response(self, response, agent, bb):
        if response.success:
            bb["capture_result"] = "succeeded"
            return Status.SUCCESS
        else:
            bb["capture_result"] = "failed"
            return Status.FAILURE


# =========================
# 4. 시작 위치로 복귀
# =========================

class ReturnToStart(ActionWithROSAction):
    """
    블랙보드['start_pose'] 로 복귀하는 Nav2 액션 노드.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        action_name = f"{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        super().__init__(name, agent, (NavigateToPose, action_name))

    def _build_goal(self, agent, bb):
        start_pose: PoseStamped = bb.get("start_pose")
        if start_pose is None:
            return None
        msg = NavigateToPose.Goal()
        msg.pose = start_pose
        return msg

    def _interpret_result(self, result, agent, bb, status_code=None):
            if status_code == GoalStatus.STATUS_SUCCEEDED:
                bb["return_result"] = "succeeded"

                # 현재 처리하던 goal 을 "완료된 goal" 로 기록
                goal_pose: PoseStamped = bb.get("goal_pose")
                if goal_pose is not None:
                    stamp = (goal_pose.header.stamp.sec, goal_pose.header.stamp.nanosec)
                    bb["completed_goal_stamp"] = stamp

                # 한 바퀴 끝났으니 goal 실행 상태 해제
                bb["goal_pose"] = None
                bb["goal_active"] = False

                return Status.SUCCESS
            else:
                bb["return_result"] = f"failed_status_{status_code}"
                # 실패 시에는 goal_active 를 어떻게 할지 고민인데,
                # 일단은 해당 goal 시나리오를 종료하는 쪽으로 동일하게 처리해도 됨.
                bb["goal_pose"] = None
                bb["goal_active"] = False
                return Status.FAILURE