// MIT License
//
// Copyright(c) 2022-2025 Matthieu Bucchianeri
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "pch.h"

#include "utils.h"
#include <log.h>
#include <util.h>

#include "trackers.h"

#include <ipc_protocol.h>

namespace openxr_api_layer {

    using namespace log;
    using namespace psvr2_toolkit::ipc;

    struct Psvr2ToolkitEyeTracker : IEyeTracker {
        Psvr2ToolkitEyeTracker() {
            WSADATA wsaData{};
            WSAStartup(MAKEWORD(2, 2), &wsaData);

            m_socket = socket(AF_INET, SOCK_STREAM, 0);
            {
                unsigned long one = 1;
                ioctlsocket(m_socket, FIONBIO, (unsigned long*)&one);
            }

            {
                struct sockaddr_in addr{};
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                addr.sin_port = htons(IPC_SERVER_PORT);

                unsigned int retries = 15;
                while (retries) {
                    if (!connect(m_socket, (struct sockaddr*)&addr, sizeof(addr))) {
                        break;
                    }
                    if (WSAGetLastError() == WSAEISCONN) {
                        break;
                    }

                    Sleep(100);
                    retries--;
                }

                if (!retries) {
                    TraceLoggingWrite(g_traceProvider, "Psvr2ToolkitEyeTracker_NotAvailable");
                    throw EyeTrackerNotSupportedException();
                }
            }

            {
#pragma pack(push, 1)
                struct {
                    CommandHeader_t header;
                    CommandDataClientRequestHandshake_t payload;
                } request{};
#pragma pack(pop)
                request.header.type = Command_ClientRequestHandshake;
                request.header.dataLen = sizeof(request.payload);
                request.payload.ipcVersion = k_unIpcVersion;
                request.payload.processId = GetCurrentProcessId();
                send(m_socket, (char*)&request, sizeof(request), 0);

#pragma pack(push, 1)
                struct {
                    CommandHeader_t header;
                    CommandDataServerHandshakeResult_t payload;
                } response{};
#pragma pack(pop)

                unsigned int retries = 5;
                char* buffer = (char*)&response;
                int offset = 0;
                while (retries) {
                    const auto result = recv(m_socket, buffer + offset, sizeof(response) - offset, 0);
                    if (result > 0) {
                        offset += result;
                    }
                    if (offset >= sizeof(response)) {
                        break;
                    }

                    Sleep(100);
                    retries--;
                }

                if (!retries || response.header.type != Command_ServerHandshakeResult ||
                    response.payload.result != HandshakeResult_Success) {
                    TraceLoggingWrite(g_traceProvider, "Psvr2ToolkitEyeTracker_NotAvailable");
                    throw EyeTrackerNotSupportedException();
                }
            }
        }

        ~Psvr2ToolkitEyeTracker() override {
            if (m_started) {
                m_listeningThread.join();
            }
            if (m_socket != INVALID_SOCKET) {
                shutdown(m_socket, 2);
                closesocket(m_socket);
                m_socket = 0;
            }
            WSACleanup();
        }

        void start(XrSession session) override {
            m_started = true;
            m_listeningThread = std::thread([&]() { ipcThread(); });
        }

        void stop() override {
        }

        bool isGazeAvailable(XrTime time) const override {
            const auto now = std::chrono::high_resolution_clock::now();
            {
                std::unique_lock lock(m_mutex);
                return (now - m_lastReceivedTime).count() < 1'000'000'000;
            }
        }

        bool getGaze(XrTime time, XrVector3f& unitVector) override {
            if (!isGazeAvailable(time)) {
                return false;
            }

            std::unique_lock lock(m_mutex);
            unitVector = m_latestGaze;
            return true;
        }

        TrackerType getType() const override {
            return TrackerType::Psvr2Toolkit;
        }

        void ipcThread() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Psvr2ToolkitEyeTracker_IpcThread");

            while (m_started) {
                struct {
                    CommandHeader_t header;
                } request{};
                request.header.type = Command_ClientRequestGazeData;
                request.header.dataLen = 0;
                send(m_socket, (char*)&request, sizeof(request), 0);

#pragma pack(push, 1)
                struct {
                    CommandHeader_t header;
                    CommandDataServerGazeDataResult_t payload;
                } response{};
#pragma pack(pop)

                unsigned int retries = 5;
                char* buffer = (char*)&response;
                int offset = 0;
                while (retries) {
                    const auto result = recv(m_socket, buffer + offset, sizeof(response) - offset, 0);
                    if (result > 0) {
                        offset += result;
                    }
                    if (offset >= sizeof(response)) {
                        break;
                    }

                    Sleep(1);
                    retries--;
                }

                if (retries && response.payload.leftEye.isGazeDirValid && response.payload.rightEye.isGazeDirValid) {
                    const auto now = std::chrono::high_resolution_clock::now();

                    XrVector3f gaze{};
                    // Average the poses from both eyes.
                    gaze.x = -(response.payload.leftEye.gazeDirNorm.x + response.payload.rightEye.gazeDirNorm.x) / 2.f;
                    gaze.y = (response.payload.leftEye.gazeDirNorm.y + response.payload.rightEye.gazeDirNorm.y) / 2.f;
                    gaze.z = -(response.payload.leftEye.gazeDirNorm.z + response.payload.rightEye.gazeDirNorm.z) / 2.f;

                    TraceLoggingWrite(g_traceProvider,
                                      "Psvr2ToolkitEyeTracker_ProcessMessage",
                                      TLArg(xr::ToString(gaze).c_str(), "EyeTrackedGazePoint"));

                    if (!(std::isnan(gaze.x) || std::isnan(gaze.y) || std::isnan(gaze.z))) {
                        std::unique_lock lock(m_mutex);
                        m_latestGaze = gaze;
                        m_lastReceivedTime = now;
                    }
                }

                // This logic is a little janky until there is a proper IPC mechanism.
                Sleep(5 - (5 - retries));
            }

            TraceLoggingWriteStop(local, "Psvr2ToolkitEyeTracker_IpcThread");
        }

        bool m_started{false};
        std::thread m_listeningThread;
        SOCKET m_socket{INVALID_SOCKET};
        mutable std::mutex m_mutex;
        XrVector3f m_latestGaze{};
        std::chrono::high_resolution_clock::time_point m_lastReceivedTime{};
    };

    std::unique_ptr<IEyeTracker> createPsvr2ToolkitEyeTracker() {
        try {
            return std::make_unique<Psvr2ToolkitEyeTracker>();
        } catch (...) {
            return {};
        }
    }

} // namespace openxr_api_layer
