#include "application/Application.h"
#include <iostream>

#ifdef MISSILESIM_PORTABLE
#include <filesystem>
#include <fstream>
#include <string>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace
{
    constexpr const char *kLogFileName = "missilesim.log";

    std::filesystem::path executableDirectory()
    {
        std::wstring buffer(MAX_PATH, L'\0');
        for (;;)
        {
            const DWORD length = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
            if (length == 0)
            {
                return {};
            }
            if (length < buffer.size())
            {
                buffer.resize(length);
                return std::filesystem::path(buffer).parent_path();
            }
            buffer.resize(buffer.size() * 2);
        }
    }

    // Shipped builds have no console and may be started from a shortcut or
    // another folder. Run from the exe's own directory so assets/, config/ and
    // imgui.ini resolve beside it, and send all log output to a file that can
    // be sent back when something goes wrong.
    class StandaloneEnvironment
    {
    public:
        StandaloneEnvironment()
        {
            const std::filesystem::path directory = executableDirectory();
            if (!directory.empty())
            {
                std::error_code error;
                std::filesystem::current_path(directory, error);
            }

            m_log.open(kLogFileName, std::ios::trunc);
            if (m_log)
            {
                m_log << std::unitbuf;
                m_previousOut = std::cout.rdbuf(m_log.rdbuf());
                m_previousErr = std::cerr.rdbuf(m_log.rdbuf());
                m_previousLog = std::clog.rdbuf(m_log.rdbuf());
            }
        }

        ~StandaloneEnvironment()
        {
            if (m_previousOut)
            {
                std::cout.rdbuf(m_previousOut);
                std::cerr.rdbuf(m_previousErr);
                std::clog.rdbuf(m_previousLog);
            }
        }

        StandaloneEnvironment(const StandaloneEnvironment &) = delete;
        StandaloneEnvironment &operator=(const StandaloneEnvironment &) = delete;

    private:
        std::ofstream m_log;
        std::streambuf *m_previousOut = nullptr;
        std::streambuf *m_previousErr = nullptr;
        std::streambuf *m_previousLog = nullptr;
    };

    void reportFatalError(const std::string &message)
    {
        const std::string text = "MissileSim stopped because of an error:\n\n" + message +
                                 "\n\nMore details are in " + kLogFileName + " next to MissileSim.exe.";
        MessageBoxA(nullptr, text.c_str(), "MissileSim", MB_OK | MB_ICONERROR);
    }
}
#endif

int main()
{
#ifdef MISSILESIM_PORTABLE
    StandaloneEnvironment environment;
#endif

    try
    {
        // The window is sized and centred from the monitor at startup; these are
        // only the pre-window defaults.
        Application app(1280, 720, "MissileSim");
        app.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
#ifdef MISSILESIM_PORTABLE
        reportFatalError(e.what());
#endif
        return -1;
    }
    catch (...)
    {
        std::cerr << "Error: unknown exception" << std::endl;
#ifdef MISSILESIM_PORTABLE
        reportFatalError("An unknown error occurred.");
#endif
        return -1;
    }

    return 0;
}
