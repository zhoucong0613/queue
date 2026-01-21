import threading
import time
from typing import Any, Callable, Optional, List
from dataclasses import dataclass
import ctypes

# 模拟C版本的队列状态枚举（对齐mqueue.h）
class QueueStatus:
    E_QUEUE_OK = 0
    E_QUEUE_ERROR_NO_MEM = 1
    E_QUEUE_ERROR_FAILED = 2
    E_QUEUE_ERROR_FULL = 3
    E_QUEUE_ERROR_TIMEOUT = 4
    E_QUEUE_ERROR_REPEAT = 5

# 模拟C版本的timestamp获取（对齐util.h）
def get_timestamp_ms() -> int:
    """获取当前时间戳（毫秒），对齐C版本的get_timestamp_ms"""
    return int(time.time() * 1000)

# 对齐C版本的data_item_t结构体
class DataItem:
    def __init__(self):
        self.is_init_added: int = 0          # 初始化添加标记
        self.item_count: int = 0             # 数据项数量
        self.items: Optional[bytearray] = None  # 存储数据的缓冲区
        self.ref_repay: int = 0              # 归还引用计数
        self.ref_obtain: int = 0             # 获取引用计数
        self.index: int = 0                  # 索引
        self.inused_update_time_ms: int = 0  # 入队时间戳（ms）
        self.unused_update_time_ms: int = 0  # 归还时间戳（ms）
        self.inused_frame_index: int = 0     # 入队帧索引
        self.unused_frame_index: int = 0     # 归还帧索引
        self.user_recorder: List[int] = [0] * 5  # 多用户读取标记（SYNC_QUEUE_MAX_USER=5）

# 对齐C版本的sync_queue_info_t结构体
@dataclass
class SyncQueueInfo:
    productor_name: str = ""
    consumer_name: str = ""
    is_need_malloc_in_advance: bool = True  # 是否预分配内存
    is_external_buffer: bool = False        # 是否使用外部缓冲区
    queue_len: int = 10                     # 队列长度（实际容量=queue_len+1）
    data_item_size: int = 0                 # 单个数据项大小（字节）
    data_item_count: int = 1                # 每个DataItem包含的数据项数量
    
    # 初始化/反初始化回调
    item_data_init_param: Any = None
    item_data_init_func: Optional[Callable[[Any, Any], int]] = None  # 返回0成功
    
    item_data_deinit_param: Any = None
    item_data_deinit_func: Optional[Callable[[Any, Any], int]] = None  # 返回0成功

# 核心同步队列类（对齐C版本的sync_queue_t）
class SyncQueue:
    def __init__(self):
        # 队列基本信息
        self.user_count: int = 0                  # 多用户数量（最大5）
        self.inused_queue_count: int = 0          # 已使用队列中的数据项数
        self.sync_queue_info: Optional[SyncQueueInfo] = None
        
        # 双队列（模拟C版本的tsQueue）
        self._unused_queue: List[DataItem] = []   # 未使用队列
        self._inused_queue: List[DataItem] = []    # 已使用队列
        self._queue_capacity: int = 0             # 队列总容量（queue_len+1）
        
        # 线程同步原语（对齐C版本的pthread_mutex/pthread_cond）
        self._unused_lock = threading.Lock()
        self._inused_lock = threading.Lock()
        self._unused_cond = threading.Condition(self._unused_lock)
        self._inused_cond = threading.Condition(self._inused_lock)

    def add_user(self) -> int:
        """添加用户，返回用户ID（0~4），超过5个返回-1（对齐C版本）"""
        with self._unused_lock:  # 加锁保证线程安全
            self.user_count += 1
            # 仅当用户数超限时报错打印
            if self.user_count > 5:  # SYNC_QUEUE_MAX_USER
                print(f"[Queue add user error]: user count exceed max(5), current: {self.user_count}")
                return -1
            return self.user_count - 1

    def create_multi_user(self, queue_info: SyncQueueInfo) -> int:
        """创建多用户队列（对齐C版本）"""
        ret = self.create(queue_info)
        if ret == 0:
            self.user_count = 0  # C版本特有的重置逻辑
        return ret

    def create(self, queue_info: SyncQueueInfo) -> int:
        """创建同步队列（对齐C版本），返回0成功，-1失败"""
        # 1. 初始化队列基础参数
        self.sync_queue_info = queue_info
        self._queue_capacity = queue_info.queue_len + 1  # 环形队列留空
        self._unused_queue.clear()
        self._inused_queue.clear()
        self.inused_queue_count = 0

        # 2. 预分配数据项（对齐C版本的循环逻辑）
        for j in range(queue_info.queue_len):
            try:
                # 2.1 分配数据缓冲区
                items = None
                if queue_info.is_need_malloc_in_advance:
                    items = bytearray(queue_info.data_item_size * queue_info.data_item_count)
                    # 调用初始化函数（对齐C版本）
                    if queue_info.item_data_init_func:
                        for k in range(queue_info.data_item_count):
                            item_data = items[k * queue_info.data_item_size : (k+1) * queue_info.data_item_size]
                            ret = queue_info.item_data_init_func(queue_info.item_data_init_param, item_data)
                            if ret != 0:
                                print("item_data_init_func failed.")
                                return -1

                # 2.2 创建DataItem对象（对齐C版本的malloc+memset）
                item = DataItem()
                item.is_init_added = 1
                item.item_count = queue_info.data_item_count
                item.items = items
                item.index = j
                item.ref_repay = 1  # 初始引用计数
                item.ref_obtain = 1

                # 2.3 入队到未使用队列（检查容量）
                with self._unused_lock:
                    if self._is_queue_full(self._unused_queue):
                        print("未使用队列已满")
                        return -1
                    self._unused_queue.append(item)
                    self._unused_cond.notify_all()

            except MemoryError:
                print("malloc failed: queue item.")
                return -1

        # 3. 初始化用户数
        self.user_count = 1
        return 0

    def _is_queue_full(self, queue: List[DataItem]) -> bool:
        """判断队列是否已满（对齐C版本的环形队列判断）"""
        return len(queue) >= self._queue_capacity - 1

    # -------------------------- 生产者接口 --------------------------
    def get_unused_object(self, timeout_ms: int = 1000) -> tuple[int, Optional[DataItem]]:
        """
        生产者获取未使用对象（对齐C版本）
        返回：(状态码, DataItem)，状态码0成功，非0失败
        """
        timeout_duration_ms = 1000  # 对齐C版本的轮询步长
        run_count = 0
        data_item = None

        with self._unused_lock:
            while timeout_duration_ms * run_count < timeout_ms:
                run_count += 1
                # 检查队列是否有数据
                if self._unused_queue:
                    data_item = self._unused_queue.pop(0)
                    self._unused_cond.notify_all()
                    return QueueStatus.E_QUEUE_OK, data_item
                
                # 每次轮询超时都打印
                print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
                      f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] "
                      f"sync_queue_get_unused_object polling timeout: run_count={run_count}, wait time={timeout_duration_ms * run_count}ms")
                
                # 等待后继续轮询（对齐C版本的usleep+循环）
                remaining = min(timeout_duration_ms / 1000.0, (timeout_ms - timeout_duration_ms*(run_count-1))/1000.0)
                self._unused_cond.wait(timeout=remaining)

        # 最终超时总提示
        print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
              f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] "
              f"sync_queue_get_unused_object final timeout (total wait: {timeout_ms}ms)")
        return QueueStatus.E_QUEUE_ERROR_TIMEOUT, None

    def _inused_queue_enqueue_process(self, data_item: DataItem) -> int:
        """入队到已使用队列的处理函数（对齐C版本）"""
        data_item.inused_frame_index += 1
        data_item.inused_update_time_ms = get_timestamp_ms()
        self.inused_queue_count += 1

        # 初始化多用户标记
        for i in range(self.user_count):
            if i < len(data_item.user_recorder):
                data_item.user_recorder[i] = 1
        return 1  # 1表示需要入队

    def save_inused_object(self, data_item: DataItem, timeout_ms: int = 1000) -> int:
        """
        生产者保存到已使用队列（对齐C版本）
        返回：0成功，-1失败（超时）
        """
        check_duration_ms = 500  # 对齐C版本的轮询步长
        timeout_ms_consume = 0
        ret = -1

        # 初始化引用计数（对齐C版本）
        data_item.is_init_added = 0
        data_item.ref_repay = self.user_count
        data_item.ref_obtain = self.user_count

        with self._inused_lock:
            while timeout_ms_consume < timeout_ms:
                # 检查队列是否已满
                if not self._is_queue_full(self._inused_queue):
                    # 执行入队处理函数
                    need_enqueue = self._inused_queue_enqueue_process(data_item)
                    if need_enqueue:
                        self._inused_queue.append(data_item)
                        self._inused_cond.notify_all()
                    ret = 0
                    break
                
                # 每次轮询队列满都打印
                time.sleep(check_duration_ms / 1000.0)
                timeout_ms_consume += check_duration_ms
                print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
                      f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] "
                      f"sync_queue_save_inused_object queue full: already wait {timeout_ms_consume}ms (total timeout: {timeout_ms}ms)")

        return ret

    # -------------------------- 消费者接口 --------------------------
    def _dequeue_process_func(self, data_item: DataItem) -> int:
        """出队处理函数（对齐C版本）"""
        if data_item.ref_obtain <= 0:
            print(f"dequeue_process_func error, found data ref count is {data_item.ref_obtain}")
            return 0  # 0表示不需要出队
        
        data_item.ref_obtain -= 1
        if data_item.ref_obtain == 0:
            self.inused_queue_count -= 1
            return 1  # 1表示需要出队
        return 0

    def _dequeue_process_func_with_user(self, data_item: DataItem, user_flag: int) -> tuple[int, bool]:
        """
        带用户标记的出队处理函数（修复版）
        返回：(状态码, 是否需要出队)
        状态码：0-成功（未重复），-1-重复数据，1-引用计数异常
        """
        # 1. 引用计数异常（报错打印）
        if data_item.ref_obtain <= 0:
            print(f"dequeue_process_func_with_user error, found data ref count is {data_item.ref_obtain}")
            return 1, False  # 状态码1：异常
        
        # 2. 检查用户是否已读取（重复数据，无打印）
        if user_flag >= len(data_item.user_recorder):
            return -1, False  # 状态码-1：无效用户/重复数据
        if data_item.user_recorder[user_flag] == 0:
            return -1, False  # 状态码-1：重复数据
        
        # 3. 标记用户已读取，递减引用计数
        data_item.user_recorder[user_flag] = 0
        data_item.ref_obtain -= 1

        # 4. 判断是否需要出队（所有用户都已读取）
        need_dequeue = (data_item.ref_obtain == 0)
        if need_dequeue:
            self.inused_queue_count -= 1
        
        return 0, need_dequeue  # 状态码0：成功

    def obtain_inused_object(self, timeout_ms: int = 1000) -> tuple[int, Optional[DataItem]]:
        """
        消费者获取已使用对象（修复版）
        返回：(状态码, DataItem)，0成功，非0失败
        """
        timeout_duration_ms = 1000
        run_count = 0
        data_item = None
        ret_status = QueueStatus.E_QUEUE_ERROR_TIMEOUT

        with self._inused_lock:
            while timeout_duration_ms * run_count < timeout_ms:
                run_count += 1
                # 遍历队列找可出队的项（对齐C版本）
                found = False
                for i, item in enumerate(self._inused_queue):
                    need_dequeue = self._dequeue_process_func(item)
                    if need_dequeue:
                        data_item = self._inused_queue.pop(i)
                        self._inused_cond.notify_all()
                        ret_status = QueueStatus.E_QUEUE_OK
                        found = True
                        break
                if found:
                    return ret_status, data_item
                
                # 每次轮询超时都打印
                print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
                      f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] "
                      f"sync_queue_obtain_inused_object polling timeout: run_count={run_count}, wait time={timeout_duration_ms * run_count}ms")
                
                # 等待后继续轮询
                remaining = min(timeout_duration_ms / 1000.0, (timeout_ms - timeout_duration_ms*(run_count-1))/1000.0)
                self._inused_cond.wait(timeout=remaining)

        # 最终超时总提示
        print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
              f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] "
              f"sync_queue_obtain_inused_object final timeout (total wait: {timeout_ms}ms)")
        return ret_status, data_item

    def obtain_inused_object_with_user(self, timeout_ms: int = 1000, user_flag: int = 0) -> tuple[int, Optional[DataItem]]:
        """
        带用户标记的消费者获取接口（修复版，对齐C版本）
        返回：(状态码, DataItem)
        状态码：
            0 - 成功获取
            4 - 超时
            5 - 重复数据（E_QUEUE_ERROR_REPEAT）
        """
        timeout_duration_ms = 1000
        run_count = 0
        data_item = None
        ret_status = QueueStatus.E_QUEUE_ERROR_TIMEOUT

        with self._inused_lock:
            while timeout_duration_ms * run_count < timeout_ms:
                run_count += 1
                # 遍历队列找可读取的项（对齐C版本）
                found = False
                for i, item in enumerate(self._inused_queue):
                    # 执行用户标记处理函数
                    status, need_dequeue = self._dequeue_process_func_with_user(item, user_flag)
                    
                    if status == 0:  # 成功（未重复）
                        data_item = item  # 返回对象（即使不需要出队）
                        ret_status = QueueStatus.E_QUEUE_OK
                        # 只有所有用户都读取完，才从队列中弹出
                        if need_dequeue:
                            self._inused_queue.pop(i)
                            self._inused_cond.notify_all()
                        found = True
                        break
                    elif status == -1:  # 重复数据（报错打印）
                        ret_status = QueueStatus.E_QUEUE_ERROR_REPEAT
                        print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
                              f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] user[{user_flag}] "
                              f"sync_queue_obtain_inused_object_with_user failed: {QueueStatus.E_QUEUE_ERROR_REPEAT} (重复数据)")
                        return ret_status, None
                
                if found:
                    return ret_status, data_item
                
                # 每次轮询超时都打印
                print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
                      f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] user[{user_flag}] "
                      f"sync_queue_obtain_inused_object_with_user polling timeout: run_count={run_count}, wait time={timeout_duration_ms * run_count}ms")
                
                # 等待后继续轮询
                remaining = min(timeout_duration_ms / 1000.0, (timeout_ms - timeout_duration_ms*(run_count-1))/1000.0)
                self._inused_cond.wait(timeout=remaining)

        # 最终超时总提示
        print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
              f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] user[{user_flag}] "
              f"sync_queue_obtain_inused_object_with_user final timeout (total wait: {timeout_ms}ms)")
        return ret_status, data_item

    def _enqueue_process_func(self, data_item: DataItem) -> int:
        """归还到未使用队列的处理函数（对齐C版本）"""
        data_item.ref_repay -= 1
        if data_item.ref_repay == 0:
            data_item.unused_update_time_ms = get_timestamp_ms()
            data_item.unused_frame_index += 1
            return 1  # 需要入队
        return 0  # 不需要入队

    def repay_unused_object(self, data_item: DataItem, timeout_ms: int = 1000) -> int:
        """
        消费者归还未使用对象（对齐C版本）
        返回：0成功，-1失败（超时）
        """
        check_duration_ms = 500
        timeout_ms_consume = 0
        ret = -1

        data_item.is_init_added = 0

        with self._unused_lock:
            while timeout_ms_consume < timeout_ms:
                # 检查队列是否已满
                if not self._is_queue_full(self._unused_queue):
                    # 执行归还处理函数
                    need_enqueue = self._enqueue_process_func(data_item)
                    if need_enqueue:
                        self._unused_queue.append(data_item)
                        self._unused_cond.notify_all()
                    ret = 0
                    break
                
                # 每次轮询队列满都打印
                time.sleep(check_duration_ms / 1000.0)
                timeout_ms_consume += check_duration_ms
                print(f"[{self.sync_queue_info.productor_name if self.sync_queue_info else ''} -> "
                      f"{self.sync_queue_info.consumer_name if self.sync_queue_info else ''}] "
                      f"sync_queue_repay_unused_object queue full: already wait {timeout_ms_consume}ms (total timeout: {timeout_ms}ms)")

        return ret

    def destroy(self) -> int:
        """销毁队列（对齐C版本），释放所有资源"""
        # 1. 清理已使用队列
        with self._inused_lock:
            while self._inused_queue:
                item = self._inused_queue.pop(0)
                # 调用反初始化函数
                if (self.sync_queue_info and self.sync_queue_info.item_data_deinit_func and
                    self.sync_queue_info.is_need_malloc_in_advance and item.items):
                    for k in range(item.item_count):
                        item_data = item.items[k * self.sync_queue_info.data_item_size : (k+1) * self.sync_queue_info.data_item_size]
                        self.sync_queue_info.item_data_deinit_func(self.sync_queue_info.item_data_deinit_param, item_data)
                # 释放缓冲区
                item.items = None
        
        # 2. 清理未使用队列
        with self._unused_lock:
            while self._unused_queue:
                item = self._unused_queue.pop(0)
                if (self.sync_queue_info and self.sync_queue_info.item_data_deinit_func and
                    self.sync_queue_info.is_need_malloc_in_advance and item.items):
                    for k in range(item.item_count):
                        item_data = item.items[k * self.sync_queue_info.data_item_size : (k+1) * self.sync_queue_info.data_item_size]
                        self.sync_queue_info.item_data_deinit_func(self.sync_queue_info.item_data_deinit_param, item_data)
                item.items = None
        
        # 3. 重置队列状态
        self.user_count = 0
        self.inused_queue_count = 0
        self.sync_queue_info = None
        self._queue_capacity = 0

        return 0