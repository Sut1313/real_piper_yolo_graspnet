"""C1 集成测试：服务回调 join 阻塞期间 /joint_states_feedback 订阅回调仍被处理。

真实 GraspExecutorNode + MultiThreadedExecutor + 真实服务触发（scan_only=false，
move_group=None 自动 fail-closed 快速失败路径——本环境未装 moveit_py）。

验证机制：流程运行期间（服务回调在 _run_flow_in_thread 内 join 阻塞）持续发布
夹爪反馈；在流程内记录 _latest_gripper。单线程 executor 下该值恒为 None
（反馈回调饿死，夹爪判定恒失败）；MultiThreadedExecutor 下其他线程处理订阅
回调，该值非 None。

rclpy 不可用时自动跳过（pytest.importorskip），无 ROS 环境不阻断其余单测。
"""

import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from real_piper_perception.grasp_executor import GraspExecutorNode

FIXTURES = Path(__file__).parent / 'fixtures'
FIXTURE_CSV = str(FIXTURES / 'sample_grasps.csv')
FIXTURE_HANDEYE = str(FIXTURES / 'handeye_fixture.yaml')


def test_service_join_does_not_starve_feedback_subscription(tmp_path):
    rclpy.init()
    node = None
    executor = None
    pub_thread = None
    stop = threading.Event()
    try:
        node = GraspExecutorNode(
            parameter_overrides=[
                Parameter('scan_only', value=False),
                Parameter('handeye_yaml', value=FIXTURE_HANDEYE),
                Parameter('shared_dir', value=str(tmp_path)),
                Parameter('csv_path', value=FIXTURE_CSV),
            ])
        # C1 修复点：MultiThreadedExecutor（与 grasp_executor.main() 一致）
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 在流程内（服务回调 join 阻塞期间）记录反馈是否已被订阅回调更新。
        # 关键：反馈只在流程开始后才发布（flow_started 事件）——若在服务触发前
        # 发布，ST/MTE 下 recorded 都为非 None，测试无法区分（判别力问题）。
        recorded: dict = {}
        flow_started = threading.Event()

        def slow_flow(trigger):
            # 模拟长流程：fail-closed 快速失败路径上加宽 join 窗口。
            # 流程内轮询等待反馈（≤3s）——MTE 下反馈回调在其他线程处理，
            # 很快得到；ST 下 join 阻塞期间无任何回调处理，必然超时取 None。
            flow_started.set()
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if node._executor._latest_gripper is not None:
                    break
                time.sleep(0.05)
            recorded['latest_gripper'] = node._executor._latest_gripper
            return {'success': True, 'message': 'smoke'}

        node._run_flow_impl = slow_flow

        def pub_loop():
            # 发布器提前创建（DDS 发现完成），但只在流程开始后发布
            pub = node.create_publisher(JointState, '/joint_states_feedback', 10)
            time.sleep(1.0)  # DDS 发现宽限
            flow_started.wait(10.0)
            while not stop.is_set():
                msg = JointState()
                msg.name = ['gripper']
                msg.position = [0.01]
                pub.publish(msg)
                time.sleep(0.01)

        pub_thread = threading.Thread(target=pub_loop, daemon=True)
        pub_thread.start()

        cli = node.create_client(Trigger, 'grasp_execute')
        assert cli.wait_for_service(5.0), 'grasp_execute 服务未就绪'
        future = cli.call_async(Trigger.Request())
        # 注意：不得再对同一 executor 调 rclpy.spin_until_future_complete——
        # Humble rclpy 不支持多线程并发 spin 同一 MultiThreadedExecutor
        # （共享 wait set 竞态会丢消息/漏唤醒）。生产模式 = 单一 spin 线程
        # （与 grasp_executor.main() 一致），这里仅轮询 future。
        deadline = time.monotonic() + 15.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert future.done(), 'grasp_execute 服务调用超时'
        assert future.result() is not None
        assert future.result().success, f'service failed: {future.result().message}'
        # C1 断言：join 期间订阅回调被其他线程处理 → 流程内反馈已更新。
        # 单线程 executor 下此值恒为 None（反馈饿死 → 夹爪判定恒失败）。
        assert recorded.get('latest_gripper') is not None, (
            '服务回调 join 期间 /joint_states_feedback 回调未被处理（C1 未修复：'
            '_latest_gripper 恒为 None，Task 9 接线后每次抓取都会在 gripper 阶段误报失败）')
    finally:
        stop.set()
        if pub_thread is not None:
            pub_thread.join(timeout=1.0)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
