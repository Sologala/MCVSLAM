#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>
namespace MCVSLAM
{
	class ThreadPool
	{
	public:
		ThreadPool(size_t);
		template <class F, class... Args>
		auto enqueue(F &&f, Args &&...args)
		    -> std::future<typename std::result_of<F(Args...)>::type>;
		~ThreadPool();

	private:
		// 线程池队列
		std::vector<std::thread> workers;
		// 任务队列 假设所有的任务都是一个 返回void 并且输入为空的函数
		std::queue<std::function<void()>> tasks;

		// 同步变量 以及条件量
		std::mutex queue_mutex;
		std::condition_variable condition;
		bool stop;
	};

	// the constructor just launches some amount of workers
	inline ThreadPool::ThreadPool(size_t threads)
	    : stop(false)
	{
		for (size_t i = 0; i < threads; ++i)
			// 初始化的时候创建 threads 个线程，每个线程执行相同的逻辑
			/* 
				1. 获取一个 任务
				2. 执行任务
			 */
			workers.emplace_back(
			    [this]
			    {
				    for (;;)
				    {
					    // 1. 获取一个 任务
					    std::function<void()> task;
					    {
						    std::unique_lock<std::mutex> lock(this->queue_mutex);
						    // 条件变量 condition 可以让程序在 给定条件失败的情况下，将自身线程陷入 阻塞，避免了使用 while 进行轮询
						    // 第二个参数 是一个 lambda 表达式
						    this->condition.wait(lock,
						                         [this]
						                         { return this->stop || !this->tasks.empty(); });

						    if (this->stop && this->tasks.empty())
							    return;
						    // 获取一个任务
						    task = std::move(this->tasks.front());
						    this->tasks.pop();
					    }
					    // 2. 执行任务
					    task();
				    }
			    });
	}

	/* 
		进队 流程
	 */
	template <class F, class... Args>
	auto ThreadPool::enqueue(F &&f, Args &&...args)
	    -> std::future<typename std::result_of<F(Args...)>::type>
	{
		/* 
			需要将 模板函数 以及参数 进行包装成一个 task
			使用 std::result_of<F (args) >::type 可以测试出 函数的返回值类型
		 */
		using return_type = typename std::result_of<F(Args...)>::type;

		// 1. 包装一个 task
		auto task = std::make_shared<std::packaged_task<return_type()>>(
		    std::bind(std::forward<F>(f), std::forward<Args>(args)...));

		//  2. task 里面包含了返回值的地址，使用 get_future 可以获得这个返回值
		std::future<return_type> res = task->get_future();
		{
			std::unique_lock<std::mutex> lock(queue_mutex);

			// don't allow enqueueing after stopping the pool
			if (stop)
				throw std::runtime_error("enqueue on stopped ThreadPool");

			tasks.emplace([task]()
			              { (*task)(); });
		}
		// 3.添加了一个线程之后 通知 worker
		condition.notify_one();
		// 4. 返回task 处理结果 的handler

		return res;
	}

	// the destructor joins all threads
	inline ThreadPool::~ThreadPool()
	{
		{
			std::unique_lock<std::mutex> lock(queue_mutex);
			stop = true;
		}
		condition.notify_all();
		for (std::thread &worker : workers)
			worker.join();
	}
} // namespace ORB_SLAM2

#endif