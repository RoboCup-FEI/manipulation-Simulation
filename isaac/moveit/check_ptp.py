"""Call the MTC test service and require a completed, verified motion."""
import rclpy
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node('lwa4p_ptp_check')
    try:
        client = node.create_client(Trigger, '/lwa4p/test_ptp')
        if not client.wait_for_service(timeout_sec=60):
            raise TimeoutError('MTC test service is unavailable')
        future = client.call_async(Trigger.Request())
        concurrent = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, concurrent, timeout_sec=30)
        if (not concurrent.done() or concurrent.result() is None or
                concurrent.result().success or concurrent.result().message != 'BUSY'):
            raise RuntimeError('Concurrent request was not rejected with BUSY')
        rclpy.spin_until_future_complete(node, future, timeout_sec=240)
        if not future.done():
            raise TimeoutError('MTC test did not complete within 240 seconds')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'MTC test failed: {response}')
        print(response.message, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
