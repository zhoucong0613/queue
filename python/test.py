import time
import threading
from stereo_queue import (
    SyncQueue,
    SyncQueueInfo,
    QueueStatus,
    get_timestamp_ms
)

# ===================== 测试用回调函数 =====================
def test_item_init(param: any, data: bytearray) -> int:
    """测试用数据项初始化函数"""
    # 填充测试数据（固定值，避免时间戳长度问题）
    fill_data = b'test_'[:len(data)]  # 固定测试数据
    data[:len(fill_data)] = fill_data
    print(f"[初始化回调] 数据项初始化完成，内容: {data}")
    return 0  # 返回0表示成功

def test_item_deinit(param: any, data: bytearray) -> int:
    """测试用数据项反初始化函数"""
    # 清空数据
    data[:] = b''
    print(f"[反初始化回调] 数据项已清空")
    return 0  # 返回0表示成功

# ===================== 测试工具函数 =====================
def print_separator(title: str):
    """打印分隔线，方便查看测试结果"""
    print("\n" + "="*60)
    print(f"【{title}】")
    print("="*60)

# ===================== 核心测试流程 =====================
def test_single_user_basic_flow():
    """测试单用户基本生产消费流程"""
    print_separator("单用户基本生产消费流程测试")
    
    # 1. 创建队列配置
    queue_config = SyncQueueInfo(
        productor_name="camera_producer",
        consumer_name="display_consumer",
        queue_len=3,          # 队列长度（实际容量4）
        data_item_size=10,    # 单个数据项10字节
        data_item_count=1,    # 每个DataItem包含1个数据项
        item_data_init_param=None,
        item_data_init_func=test_item_init,
        item_data_deinit_param=None,
        item_data_deinit_func=test_item_deinit
    )
    
    # 2. 初始化队列
    sync_queue = SyncQueue()
    create_ret = sync_queue.create(queue_config)
    assert create_ret == 0, f"队列创建失败，返回值: {create_ret}"
    print("✅ 队列创建成功")
    
    # 3. 生产者：获取未使用对象
    status, unused_item = sync_queue.get_unused_object(timeout_ms=1000)
    assert status == QueueStatus.E_QUEUE_OK and unused_item is not None, f"获取未使用对象失败，状态码: {status}"
    print(f"✅ 生产者获取未使用对象成功，对象索引: {unused_item.index}")
    
    # 4. 生产者：保存到已使用队列
    save_ret = sync_queue.save_inused_object(unused_item, timeout_ms=1000)
    assert save_ret == 0, f"保存到已使用队列失败，返回值: {save_ret}"
    print("✅ 生产者保存对象到已使用队列成功")
    
    # 5. 消费者：获取已使用对象
    status, inused_item = sync_queue.obtain_inused_object(timeout_ms=1000)
    assert status == QueueStatus.E_QUEUE_OK and inused_item is not None, f"消费者获取对象失败，状态码: {status}"
    print(f"✅ 消费者获取已使用对象成功，数据内容: {inused_item.items}")
    
    # 6. 消费者：归还对象到未使用队列
    repay_ret = sync_queue.repay_unused_object(inused_item, timeout_ms=1000)
    assert repay_ret == 0, f"归还对象失败，返回值: {repay_ret}"
    print("✅ 消费者归还对象到未使用队列成功")
    
    # 7. 销毁队列
    destroy_ret = sync_queue.destroy()
    assert destroy_ret == 0, f"队列销毁失败，返回值: {destroy_ret}"
    print("✅ 队列销毁成功")

def test_multi_user_scenario():
    """测试多用户场景（核心功能）"""
    print_separator("多用户生产消费流程测试")
    
    # 1. 创建队列配置
    queue_config = SyncQueueInfo(
        productor_name="sensor_producer",
        consumer_name="multi_consumer",
        queue_len=5,
        data_item_size=8,
        data_item_count=1,
        item_data_init_func=test_item_init,
        item_data_deinit_func=test_item_deinit
    )
    
    # 2. 初始化多用户队列
    sync_queue = SyncQueue()
    create_ret = sync_queue.create_multi_user(queue_config)
    assert create_ret == 0, f"多用户队列创建失败，返回值: {create_ret}"
    print("✅ 多用户队列创建成功")
    
    # 3. 添加2个用户（总用户数3）
    user1_id = sync_queue.add_user()
    user2_id = sync_queue.add_user()
    assert user1_id == 0 and user2_id == 1, f"添加用户失败，用户ID: {user1_id}, {user2_id}"
    print(f"✅ 添加2个用户成功，用户ID: {user1_id}, {user2_id}（总用户数: {sync_queue.user_count}）")
    
    # 4. 生产者生产数据
    status, item = sync_queue.get_unused_object(timeout_ms=1000)
    assert status == QueueStatus.E_QUEUE_OK and item is not None, "生产者获取对象失败"
    save_ret = sync_queue.save_inused_object(item, timeout_ms=1000)
    assert save_ret == 0, "生产者保存对象失败"
    print("✅ 生产者生产数据成功")
    
    # 5. 多用户消费数据（带用户标记）
    # 用户1获取数据
    status, user1_item = sync_queue.obtain_inused_object_with_user(
        timeout_ms=1000, user_flag=user1_id
    )
    assert status == QueueStatus.E_QUEUE_OK and user1_item is not None, f"用户{user1_id}获取数据失败，状态码: {status}"
    print(f"✅ 用户{user1_id}获取数据成功，引用计数: {user1_item.ref_obtain}")
    
    # 用户2获取数据
    status, user2_item = sync_queue.obtain_inused_object_with_user(
        timeout_ms=1000, user_flag=user2_id
    )
    assert status == QueueStatus.E_QUEUE_OK and user2_item is not None, f"用户{user2_id}获取数据失败，状态码: {status}"
    print(f"✅ 用户{user2_id}获取数据成功，引用计数: {user2_item.ref_obtain}")
    
    # 验证：引用计数为0（所有用户都已获取）
    assert user2_item.ref_obtain == 0, f"引用计数异常，预期0，实际: {user2_item.ref_obtain}"
    
    # 6. 归还对象（每个用户都需要归还）
    repay_ret1 = sync_queue.repay_unused_object(user1_item, timeout_ms=1000)
    repay_ret2 = sync_queue.repay_unused_object(user2_item, timeout_ms=1000)
    assert repay_ret1 == 0 and repay_ret2 == 0, "归还对象失败"
    print("✅ 所有用户归还对象成功")
    
    # 7. 销毁队列
    sync_queue.destroy()
    print("✅ 多用户队列销毁成功")

def test_timeout_scenario():
    """测试超时场景（队列空/满时的超时处理）"""
    print_separator("超时场景测试")
    
    # 1. 创建队列（长度2，容量3，未使用队列初始有2个对象）
    queue_config = SyncQueueInfo(
        productor_name="timeout_producer",
        consumer_name="timeout_consumer",
        queue_len=2,
        data_item_size=5,
        data_item_count=1
    )
    sync_queue = SyncQueue()
    sync_queue.create(queue_config)
    
    # 2. 测试1：消费者获取空队列（超时）
    status, item = sync_queue.obtain_inused_object(timeout_ms=500)
    assert status == QueueStatus.E_QUEUE_ERROR_TIMEOUT, f"超时测试失败，状态码: {status}"
    print("✅ 消费者获取空队列超时测试通过")
    
    # 3. 测试2：生产者填满已使用队列后再次保存（超时）
    # 步骤1：获取2个未使用对象（未使用队列空）
    status1, item1 = sync_queue.get_unused_object(timeout_ms=1000)
    status2, item2 = sync_queue.get_unused_object(timeout_ms=1000)
    assert status1 == QueueStatus.E_QUEUE_OK and status2 == QueueStatus.E_QUEUE_OK, "获取未使用对象失败"
    
    # 步骤2：保存2个对象到已使用队列（已使用队列满，queue_len=2）
    sync_queue.save_inused_object(item1, timeout_ms=1000)
    sync_queue.save_inused_object(item2, timeout_ms=1000)
    
    # 步骤3：再次尝试获取未使用对象（预期超时，因为未使用队列空）
    status3, item3 = sync_queue.get_unused_object(timeout_ms=500)
    assert status3 == QueueStatus.E_QUEUE_ERROR_TIMEOUT, f"未使用队列空超时测试失败，状态码: {status3}"
    print("✅ 生产者获取空未使用队列超时测试通过")
    
    # 步骤4：模拟生产者有一个待保存的对象（测试保存超时）
    save_ret = sync_queue.save_inused_object(item1, timeout_ms=500)
    assert save_ret == -1, f"队列满超时测试失败，返回值: {save_ret}"
    print("✅ 生产者保存到满队列超时测试通过")
    
    # 4. 销毁队列
    sync_queue.destroy()

def test_multi_thread_scenario():
    """测试多线程生产消费（模拟真实场景）"""
    print_separator("多线程生产消费测试")
    
    # 1. 初始化队列
    queue_config = SyncQueueInfo(
        productor_name="thread_producer",
        consumer_name="thread_consumer",
        queue_len=5,
        data_item_size=10,
        data_item_count=1,
        item_data_init_func=test_item_init
    )
    sync_queue = SyncQueue()
    sync_queue.create(queue_config)
    
    # 2. 定义生产者线程函数
    def producer_worker(queue: SyncQueue, produce_count: int):
        """生产者线程：生产指定数量的对象"""
        for i in range(produce_count):
            # 获取未使用对象
            status, item = queue.get_unused_object(timeout_ms=2000)
            if status != QueueStatus.E_QUEUE_OK or item is None:
                print(f"❌ 生产者{i}获取对象失败，状态码: {status}")
                continue
            
            # 模拟数据处理（写入线程ID）
            thread_id = str(threading.current_thread().ident)[:len(item.items)]
            item.items[:len(thread_id)] = thread_id.encode('utf-8')
            
            # 保存到已使用队列
            save_ret = queue.save_inused_object(item, timeout_ms=2000)
            if save_ret != 0:
                print(f"❌ 生产者{i}保存对象失败，返回值: {save_ret}")
                continue
            
            print(f"📤 生产者{i}生产数据成功，线程ID: {thread_id}")
            time.sleep(0.1)  # 模拟生产耗时
    
    # 3. 定义消费者线程函数
    def consumer_worker(queue: SyncQueue, consume_count: int, user_id: int):
        """消费者线程：消费指定数量的对象"""
        for i in range(consume_count):
            # 获取已使用对象（带用户标记）
            status, item = queue.obtain_inused_object_with_user(
                timeout_ms=2000, user_flag=user_id
            )
            if status == QueueStatus.E_QUEUE_ERROR_TIMEOUT:
                print(f"❌ 消费者{user_id}-{i}超时，未获取到数据")
                continue
            elif status == QueueStatus.E_QUEUE_ERROR_REPEAT:
                print(f"⚠️  消费者{user_id}-{i}获取到重复数据")
                continue
            elif status != QueueStatus.E_QUEUE_OK or item is None:
                print(f"❌ 消费者{user_id}-{i}获取数据失败，状态码: {status}")
                continue
            
            # 模拟数据处理
            data_str = item.items.decode('utf-8').strip('\x00')
            print(f"📥 消费者{user_id}-{i}消费数据成功，内容: {data_str}")
            
            # 归还对象
            repay_ret = queue.repay_unused_object(item, timeout_ms=2000)
            if repay_ret != 0:
                print(f"❌ 消费者{user_id}-{i}归还对象失败，返回值: {repay_ret}")
            
            time.sleep(0.15)  # 模拟消费耗时
    
    # 4. 创建并启动线程
    produce_count = 3  # 每个生产者生产3个
    consume_count = 3  # 每个消费者消费3个
    
    # 添加2个消费者用户
    user1 = sync_queue.add_user()
    user2 = sync_queue.add_user()
    
    # 启动1个生产者线程，2个消费者线程
    producer_thread = threading.Thread(
        target=producer_worker, args=(sync_queue, produce_count)
    )
    consumer1_thread = threading.Thread(
        target=consumer_worker, args=(sync_queue, consume_count, user1)
    )
    consumer2_thread = threading.Thread(
        target=consumer_worker, args=(sync_queue, consume_count, user2)
    )
    
    # 启动线程
    producer_thread.start()
    consumer1_thread.start()
    consumer2_thread.start()
    
    # 等待线程结束
    producer_thread.join()
    consumer1_thread.join()
    consumer2_thread.join()
    
    # 5. 销毁队列
    sync_queue.destroy()
    print("✅ 多线程生产消费测试完成")

# ===================== 主函数 =====================
if __name__ == "__main__":
    print("开始测试同步队列（stereo_queue.py）...")
    
    # 执行所有测试用例
    try:
        test_single_user_basic_flow()       # 单用户基本流程
        test_multi_user_scenario()          # 多用户场景
        test_timeout_scenario()             # 超时场景
        test_multi_thread_scenario()        # 多线程场景
        
        print_separator("所有测试用例执行完成")
        print("🎉 全部测试通过！")
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()  # 打印详细异常栈