#ifndef UTILS_COMMON_H_
#define UTILS_COMMON_H_

#include <string>
#include <random>
#include <sstream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>

namespace hightorque
{
    namespace utils
    {

        /**
         * @brief 生成UUID v4格式的字符串
         * @return UUID字符串，格式：xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
         */
        inline std::string generateUUID()
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, 15);

            std::stringstream ss;
            ss << std::hex << std::setfill('0');

            for (int i = 0; i < 8; i++)
                ss << std::setw(1) << dis(gen);
            ss << "-";
            for (int i = 0; i < 4; i++)
                ss << std::setw(1) << dis(gen);
            ss << "-4";
            for (int i = 0; i < 3; i++)
                ss << std::setw(1) << dis(gen);
            ss << "-";
            ss << std::setw(1) << (dis(gen) & 0x3 | 0x8);
            for (int i = 0; i < 3; i++)
                ss << std::setw(1) << dis(gen);
            ss << "-";
            for (int i = 0; i < 12; i++)
                ss << std::setw(1) << dis(gen);

            return ss.str();
        }

        /**
         * @brief 将字符串转换为大写
         */
        inline std::string toUpper(const std::string& str)
        {
            std::string result = str;
            for (char& c : result)
                c = std::toupper(c);
            return result;
        }

        /**
         * @brief 将字符串转换为小写
         */
        inline std::string toLower(const std::string& str)
        {
            std::string result = str;
            for (char& c : result)
                c = std::tolower(c);
            return result;
        }

        /**
         * @brief 去除字符串首尾空白字符
         */
        inline std::string trim(const std::string& str)
        {
            size_t first = str.find_first_not_of(" \t\n\r");
            if (first == std::string::npos)
                return "";
            size_t last = str.find_last_not_of(" \t\n\r");
            return str.substr(first, (last - first + 1));
        }

        /**
         * @brief 分割字符串
         */
        inline std::vector<std::string> split(const std::string& str, char delimiter)
        {
            std::vector<std::string> tokens;
            std::stringstream ss(str);
            std::string token;
            while (std::getline(ss, token, delimiter))
                tokens.push_back(token);
            return tokens;
        }

        /**
         * @brief 获取当前时间戳（秒，double精度）
         */
        inline double getCurrentTimeInSeconds()
        {
            return std::chrono::duration<double>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        /**
         * @brief 获取当前时间戳（毫秒）
         */
        inline int64_t getCurrentTimeMs()
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        /**
         * @brief 获取当前时间戳（微秒）
         */
        inline int64_t getCurrentTimeUs()
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        // ======================== 插值函数 ========================

        /**
         * @brief 线性插值（位置）
         */
        inline double linearTransitionPosition(double startValue, double targetValue,
                                               double startTime, double currentTime, double duration)
        {
            double t = std::min(1.0, (currentTime - startTime) / duration);
            return startValue + t * (targetValue - startValue);
        }

        /**
         * @brief 线性插值（速度）
         */
        inline double linearTransitionVelocity(double startValue, double targetValue,
                                               double startTime, double currentTime, double duration)
        {
            double t = std::min(1.0, (currentTime - startTime) / duration);
            if (t >= 1.0)
                return 0.0;
            return (targetValue - startValue) / duration;
        }

        /**
         * @brief 缓入插值（位置）
         */
        inline double easeInTransitionPosition(double startValue, double targetValue,
                                               double startTime, double currentTime, double duration)
        {
            double t = std::min(1.0, (currentTime - startTime) / duration);
            return startValue + t * t * (targetValue - startValue);
        }

        /**
         * @brief 缓出插值（位置）
         */
        inline double easeOutTransitionPosition(double startValue, double targetValue,
                                                double startTime, double currentTime, double duration)
        {
            double t = std::min(1.0, (currentTime - startTime) / duration);
            return startValue + (2.0 - t) * t * (targetValue - startValue);
        }

        /**
         * @brief 平滑插值（位置，S型曲线）
         */
        inline double smoothStepTransitionPosition(double startValue, double targetValue,
                                                   double startTime, double currentTime, double duration)
        {
            double t = std::min(1.0, (currentTime - startTime) / duration);
            double s = t * t * (3.0 - 2.0 * t);
            return startValue + s * (targetValue - startValue);
        }

        // ======================== 字符串工具 ========================

        /**
         * @brief 将 vector 转为字符串
         */
        template <typename T>
        inline std::string toString(const std::vector<T>& vec)
        {
            std::stringstream ss;
            for (size_t i = 0; i < vec.size(); ++i)
            {
                ss << vec[i];
                if (i < vec.size() - 1)
                    ss << ", ";
            }
            ss << "\n";
            return ss.str();
        }

        /**
         * @brief 处理后缀字符串，统一格式
         */
        inline std::string processSuffix(const std::string& suffix)
        {
            if (suffix.empty())
                return "";
            std::string s = suffix;
            while (!s.empty() && s.front() == '.')
                s = s.substr(1);
            if (s.empty())
                return "";
            return "." + s;
        }

        /**
         * @brief 给文件路径替换或添加后缀
         */
        inline std::string replaceOrAddSuffix(const std::string& path, const std::string& suffix)
        {
            std::filesystem::path p(path);
            return (p.parent_path() / p.stem()).string() + processSuffix(suffix);
        }

    } // namespace utils
} // namespace hightorque

#endif // UTILS_COMMON_H_
