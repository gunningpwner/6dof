#pragma once
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <mutex>
#include <Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void startNewSession() {
        std::lock_guard<std::mutex> lock(file_mtx);
        // Close existing files
        for (auto& pair : files) {
            if (pair.second.is_open()) pair.second.close();
        }
        files.clear();

        // Create new directory based on time
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << "logs/" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        session_dir = oss.str();
        
        std::filesystem::create_directories(session_dir);
        std::cout << "[Logger] Started new session: " << session_dir << std::endl;
    }

    template <typename Derived>
    void log(const std::string& name, const Eigen::MatrixBase<Derived>& data, uint64_t timestamp) {
        LogEntry entry;
        entry.name = name;
        entry.timestamp = timestamp;
        // Copy data to MatrixXf (dynamic size float matrix) to store in queue
        entry.data = data.template cast<float>();

        {
            std::lock_guard<std::mutex> lock(queue_mtx);
            log_queue.push(std::move(entry));
        }
        queue_cv.notify_one();
    }
    void log(const std::string& name, float data, uint64_t timestamp) {
        LogEntry entry;
        entry.name = name;
        entry.timestamp = timestamp;
        entry.data.resize(1, 1);
        entry.data(0, 0) = data;

        {
            std::lock_guard<std::mutex> lock(queue_mtx);
            log_queue.push(std::move(entry));
        }
        queue_cv.notify_one();
    }
private:
    Logger() : running(true) {
        startNewSession();
        worker_thread = std::thread(&Logger::worker, this);
    }
    
    ~Logger() {
        running = false;
        queue_cv.notify_all();
        if (worker_thread.joinable()) worker_thread.join();
        // Files close automatically when map is destroyed
    }

    void ensureFileOpen(const std::string& name) {
        if (files.find(name) == files.end()) {
            std::string path = session_dir  +"/"+  name  +".csv";
            files[name].open(path);
            if (!files[name].is_open()) {
                std::cerr << "[Logger] Failed to open file: " << path << std::endl;
            }
        }
    }

    struct LogEntry {
        std::string name;
        uint64_t timestamp;
        Eigen::MatrixXf data;
    };

    void worker() {
        while (running || !isQueueEmpty()) {
            std::unique_lock<std::mutex> lock(queue_mtx);
            queue_cv.wait(lock, [this] { return !log_queue.empty() || !running; });

            while (!log_queue.empty()) {
                LogEntry entry = std::move(log_queue.front());
                log_queue.pop();
                lock.unlock();

                // File I/O happens here, off the main thread
                std::lock_guard<std::mutex> file_lock(file_mtx);
                ensureFileOpen(entry.name);
                files[entry.name] << entry.timestamp << "," << entry.data.rows() << "," << entry.data.cols();
                
                if (entry.data.size() > 0) {
                    static const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",");
                    files[entry.name] << "," << entry.data.format(CSVFormat);
                }
                files[entry.name] << "\n"; // Use \n instead of endl to avoid forced flush

                lock.lock();
            }
        }
    }

    bool isQueueEmpty() {
        std::lock_guard<std::mutex> lock(queue_mtx);
        return log_queue.empty();
    }

    std::string session_dir;
    std::map<std::string, std::ofstream> files;
    std::mutex file_mtx;

    // Async Queue
    std::queue<LogEntry> log_queue;
    std::mutex queue_mtx;
    std::condition_variable queue_cv;
    std::atomic<bool> running;
    std::thread worker_thread;
};
