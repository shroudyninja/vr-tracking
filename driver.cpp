#include <openvr_driver.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <json.hpp>
#include <string>
#include <vector>
#include <array>

#pragma comment(lib, "Ws2_32.lib")

using namespace vr;
using json = nlohmann::json;

// Define missing constants/enums that might not be in your OpenVR version
// These definitions should be outside any class to be accessible globally
#ifndef IVRControllerComponent_Version
const char* const IVRControllerComponent_Version = "IVRControllerComponent_001";
#endif


// Forward declaration for our driver instance
class ServerDriver;
ServerDriver* g_serverDriverHost = nullptr;

// Simple logging function
void DebugLog(const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsprintf_s(buffer, format, args);
    va_end(args);
    OutputDebugStringA(buffer);
}

// Number of bones in the hand skeleton
const int HAND_BONE_COUNT = 31;

class PhoneController : public ITrackedDeviceServerDriver {
public:
    PhoneController(int role) : _role(role) {
        // Initialize the skeletal reference transforms
        InitSkeletalSystem();
    }

    EVRInitError Activate(uint32_t unObjectId) override {
        _objectId = unObjectId;

        // Set controller properties
        VRProperties()->SetStringProperty(
            _objectId,
            Prop_SerialNumber_String,
            _role == 0 ? "PhoneController_Left" : "PhoneController_Right"
        );
        VRProperties()->SetStringProperty(
            _objectId,
            Prop_ModelNumber_String,
            "Phone Controller"
        );

        // Set controller role
        VRProperties()->SetInt32Property(
            _objectId,
            Prop_ControllerRoleHint_Int32,
            _role == 0 ? TrackedControllerRole_LeftHand : TrackedControllerRole_RightHand
        );

        // Set controller input profile
        VRProperties()->SetStringProperty(
            _objectId,
            Prop_InputProfilePath_String,
            "{handtracking}/input/handtracking_profile.json"
        );

        // Register controller as being hand tracked
        VRProperties()->SetInt32Property(
            _objectId,
            Prop_ControllerHandSelectionPriority_Int32,
            INT32_MAX
        );

        // Set which hand this is
        VRProperties()->SetInt32Property(
            _objectId,
            Prop_Axis0Type_Int32,
            k_eControllerAxis_TrackPad
        );

        // Create skeletal components
        CreateSkeletalComponents();

        DebugLog("PhoneController %s activated with ID %u\n",
            _role == 0 ? "Left" : "Right", unObjectId);

        return VRInitError_None;
    }

    void Deactivate() override {
        DebugLog("PhoneController %s deactivated\n", _role == 0 ? "Left" : "Right");
        _objectId = k_unTrackedDeviceIndexInvalid;
    }

    void EnterStandby() override {}

    void* GetComponent(const char* pchComponentNameAndVersion) override {
        // Return this as IVRControllerComponent if requested
        if (!_stricmp(pchComponentNameAndVersion, IVRControllerComponent_Version)) {
            return static_cast<void*>(this);
        }

        // Return this as IVRDriverInput if requested
        if (!_stricmp(pchComponentNameAndVersion, IVRDriverInput_Version)) {
            return static_cast<void*>(this);
        }

        return nullptr;
    }

    void DebugRequest(const char*, char* pchResponseBuffer, uint32_t) override {}

    DriverPose_t GetPose() override {
        DriverPose_t pose = { 0 };
        pose.result = TrackingResult_Running_OK;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;

        // Assign position
        pose.vecPosition[0] = _pos[0];
        pose.vecPosition[1] = _pos[1];
        pose.vecPosition[2] = _pos[2];

        // Assign rotation quaternion
        pose.qRotation.w = _rot[3];
        pose.qRotation.x = _rot[0];
        pose.qRotation.y = _rot[1];
        pose.qRotation.z = _rot[2];

        return pose;
    }

    void UpdatePose() {
        if (_objectId != k_unTrackedDeviceIndexInvalid) {
            DriverPose_t p = GetPose();
            VRServerDriverHost()->TrackedDevicePoseUpdated(
                _objectId, p, sizeof(p)
            );
        }
    }

    // Controller state - for IVRControllerComponent compatibility
    VRControllerState_t GetControllerState() {
        return _controllerState;
    }

    bool TriggerHapticPulse(uint32_t unAxisId, uint16_t usPulseDurationMicroseconds) {
        // No haptics support for hand tracking
        return false;
    }

    // Driver input methods - for IVRDriverInput compatibility
    EVRInputError CreateBooleanComponent(PropertyContainerHandle_t ulContainer, const char* pchName, VRInputComponentHandle_t* pHandle) {
        if (!pHandle)
            return VRInputError_InvalidParam;

        // Store component handle
        *pHandle = _inputComponentHandles.size();
        _inputComponentHandles.push_back(*pHandle);

        DebugLog("Created boolean component %s with handle %llu\n", pchName, *pHandle);
        return VRInputError_None;
    }

    EVRInputError UpdateBooleanComponent(VRInputComponentHandle_t ulComponent, bool bNewValue, double fTimeOffset) {
        return VRInputError_None;
    }

    EVRInputError CreateScalarComponent(PropertyContainerHandle_t ulContainer, const char* pchName, VRInputComponentHandle_t* pHandle,
        EVRScalarType eType, EVRScalarUnits eUnits) {
        if (!pHandle)
            return VRInputError_InvalidParam;

        // Store component handle
        *pHandle = _inputComponentHandles.size();
        _inputComponentHandles.push_back(*pHandle);

        DebugLog("Created scalar component %s with handle %llu\n", pchName, *pHandle);
        return VRInputError_None;
    }

    EVRInputError UpdateScalarComponent(VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset) {
        return VRInputError_None;
    }

    EVRInputError CreateHapticComponent(PropertyContainerHandle_t ulContainer, const char* pchName, VRInputComponentHandle_t* pHandle) {
        if (!pHandle)
            return VRInputError_InvalidParam;

        // Store component handle
        *pHandle = _inputComponentHandles.size();
        _inputComponentHandles.push_back(*pHandle);

        DebugLog("Created haptic component %s with handle %llu\n", pchName, *pHandle);
        return VRInputError_None;
    }

    EVRInputError CreateSkeletonComponent(PropertyContainerHandle_t ulContainer, const char* pchName, const char* pchSkeletonPath,
        const char* pchBasePosePath, EVRSkeletalTrackingLevel eSkeletalTrackingLevel,
        const VRBoneTransform_t* pGripLimitTransforms, uint32_t unGripLimitTransformCount,
        VRInputComponentHandle_t* pHandle) {
        if (!pHandle)
            return VRInputError_InvalidParam;

        // Store skeletal component info
        _skeletalComponentName = pchName;
        _skeletalTrackingLevel = eSkeletalTrackingLevel;

        // Store component handle
        *pHandle = _inputComponentHandles.size();
        _inputComponentHandles.push_back(*pHandle);
        _skeletalComponentHandle = *pHandle;

        DebugLog("Created skeletal component %s with handle %llu\n", pchName, *pHandle);
        return VRInputError_None;
    }

    EVRInputError UpdateSkeletonComponent(VRInputComponentHandle_t ulComponent, EVRSkeletalMotionRange eMotionRange,
        const VRBoneTransform_t* pTransforms, uint32_t unTransformCount) {
        if (ulComponent != _skeletalComponentHandle || !pTransforms || unTransformCount != HAND_BONE_COUNT)
            return VRInputError_InvalidParam;

        // Copy the transforms
        memcpy(_boneTransforms.data(), pTransforms, sizeof(VRBoneTransform_t) * HAND_BONE_COUNT);

        return VRInputError_None;
    }

    // Called from the driver's RunFrame
    void SetRawPose(const std::vector<float>& pos, const std::vector<float>& rot) {
        if (pos.size() == 3) {
            _pos[0] = pos[0]; _pos[1] = pos[1]; _pos[2] = pos[2];
        }
        if (rot.size() == 4) {
            _rot[0] = rot[0]; _rot[1] = rot[1];
            _rot[2] = rot[2]; _rot[3] = rot[3];
        }
    }

    // Process hand landmark data for skeletal input
    void UpdateHandLandmarks(const std::vector<std::vector<float>>& landmarks) {
        if (landmarks.empty()) return;

        // Store hand visibility
        _handVisible = true;

        // Convert MediaPipe landmarks to VR bone transforms
        ConvertLandmarksToSkeleton(landmarks);

        if (_objectId != k_unTrackedDeviceIndexInvalid && _skeletalComponentHandle != vr::k_ulInvalidInputComponentHandle) {
            // Get the VRDriverInput interface (needed to send input updates)
            IVRDriverInput* pDriverInput = VRDriverInput();
            if (pDriverInput) { // Check if we got the interface successfully
                // Update skeletal data in SteamVR using the populated _boneTransforms
                pDriverInput->UpdateSkeletonComponent(
                    _skeletalComponentHandle,                 // Handle for the skeleton component
                    vr::VRSkeletalMotionRange_WithController,// Motion range (using official enum)
                    _boneTransforms.data(),                   // Pointer to the bone transform data
                    (uint32_t)_boneTransforms.size()          // Number of bones
                );
            }
        }
        // ***** END OF THE CODE BLOCK YOU ASKED ABOUT *****
    } // End of UpdateHandLandmarks method

private:
    uint32_t _objectId = k_unTrackedDeviceIndexInvalid;
    int _role;  // 0 = left, 1 = right
    float _pos[3] = { 0 };
    float _rot[4] = { 0, 0, 0, 1 };
    bool _handVisible = false;

    // Controller state
    VRControllerState_t _controllerState = {};

    // Skeletal tracking
    std::array<VRBoneTransform_t, HAND_BONE_COUNT> _boneTransforms;
    std::string _skeletalComponentName;
    EVRSkeletalTrackingLevel _skeletalTrackingLevel = VRSkeletalTracking_Partial;
    std::vector<VRInputComponentHandle_t> _inputComponentHandles;
    VRInputComponentHandle_t _skeletalComponentHandle = k_ulInvalidInputComponentHandle;

    // Initialize skeletal reference transforms
    void InitSkeletalSystem() {
        // Initialize with reasonable default positions
        for (int i = 0; i < HAND_BONE_COUNT; i++) {
            _boneTransforms[i].position.v[0] = 0.0f;
            _boneTransforms[i].position.v[1] = 0.0f;
            _boneTransforms[i].position.v[2] = 0.0f;

            _boneTransforms[i].orientation.w = 1.0f;
            _boneTransforms[i].orientation.x = 0.0f;
            _boneTransforms[i].orientation.y = 0.0f;
            _boneTransforms[i].orientation.z = 0.0f;
        }
    }

    // Create the skeletal component
    void CreateSkeletalComponents() {
        if (_objectId == k_unTrackedDeviceIndexInvalid) return;

        PropertyContainerHandle_t container = VRProperties()->TrackedDeviceToPropertyContainer(_objectId);
        VRDriverInput()->CreateSkeletonComponent(
            container,
            _role == 0 ? "/input/skeleton/left" : "/input/skeleton/right",
            _role == 0 ? "/skeleton/hand/left" : "/skeleton/hand/right",
            _role == 0 ? "/pose/raw/left" : "/pose/raw/right",
            VRSkeletalTracking_Partial,
            nullptr,  // Grip limit transforms
            0,        // Grip limit transform count
            &_skeletalComponentHandle
        );
    }

    // Convert MediaPipe landmarks to SteamVR bone transforms
    void ConvertLandmarksToSkeleton(const std::vector<std::vector<float>>& landmarks) {
        if (landmarks.size() < 21) return;  // MediaPipe tracks 21 hand landmarks

        // MediaPipe hand landmark indices:
        // 0: Wrist
        // 1-4: Thumb (1=CMC, 2=MCP, 3=IP, 4=TIP)
        // 5-8: Index (5=MCP, 6=PIP, 7=DIP, 8=TIP)
        // 9-12: Middle (9=MCP, 10=PIP, 11=DIP, 12=TIP)
        // 13-16: Ring (13=MCP, 14=PIP, 15=DIP, 16=TIP)
        // 17-20: Pinky (17=MCP, 18=PIP, 19=DIP, 20=TIP)

        // Scale factor to convert from MediaPipe's normalized coordinates to meters
        float scale = 0.1f;

        // Map wrist position (root)
        VRBoneTransform_t& wrist = _boneTransforms[0];
        wrist.position.v[0] = landmarks[0][0] * scale;
        wrist.position.v[1] = landmarks[0][1] * scale;
        wrist.position.v[2] = landmarks[0][2] * scale;

        // Simple mapping of landmarks to bone transforms
        // This is a simplification - a real implementation would compute proper bone orientations
        for (int i = 1; i < 21 && i < HAND_BONE_COUNT; i++) {
            _boneTransforms[i].position.v[0] = landmarks[i][0] * scale;
            _boneTransforms[i].position.v[1] = landmarks[i][1] * scale;
            _boneTransforms[i].position.v[2] = landmarks[i][2] * scale;

            // Calculate bone orientation - simplified approach
            // For a proper implementation, you'd compute the orientation from parent to child bone
            if (i > 0) {
                // Simple way to align bones - not anatomically correct but shows the concept
                int parent = GetParentBoneIndex(i);
                if (parent >= 0) {
                    // Simple direction vector between parent and current bone
                    float dx = _boneTransforms[i].position.v[0] - _boneTransforms[parent].position.v[0];
                    float dy = _boneTransforms[i].position.v[1] - _boneTransforms[parent].position.v[1];
                    float dz = _boneTransforms[i].position.v[2] - _boneTransforms[parent].position.v[2];

                    // Normalize
                    float len = sqrt(dx * dx + dy * dy + dz * dz);
                    if (len > 0.0001f) {
                        dx /= len;
                        dy /= len;
                        dz /= len;
                    }

                    // Very simplified quaternion from direction vector
                    // This is just a placeholder - proper bone orientation is more complex
                    _boneTransforms[i].orientation.w = 1.0f;
                    _boneTransforms[i].orientation.x = 0.0f;
                    _boneTransforms[i].orientation.y = 0.0f;
                    _boneTransforms[i].orientation.z = 0.0f;
                }
            }
        }
    }

    // Get parent bone index for a given bone
    int GetParentBoneIndex(int boneIndex) {
        // Very simplified parent mapping
        // A real implementation would use SteamVR's bone hierarchy
        if (boneIndex == 0) return -1;  // Wrist has no parent

        // Thumb
        if (boneIndex >= 1 && boneIndex <= 4) {
            return boneIndex - 1;  // Parent is previous bone
        }

        // Fingers
        if (boneIndex >= 5) {
            int finger = (boneIndex - 5) / 4;
            int bone = (boneIndex - 5) % 4;

            if (bone == 0) return 0;  // MCP connects to wrist
            return boneIndex - 1;     // Otherwise, parent is previous bone
        }

        return -1;
    }
};

// Globals for the two controllers and socket
std::unique_ptr<PhoneController> g_left, g_right;
SOCKET g_sock = INVALID_SOCKET;

class ServerDriver : public IServerTrackedDeviceProvider {
public:
    EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

        DebugLog("ServerDriver initializing...\n");

        // Instantiate controllers
        g_left.reset(new PhoneController(0));
        g_right.reset(new PhoneController(1));

        // Register controllers with the system
        VRServerDriverHost()->TrackedDeviceAdded(
            "PhoneController_Left",
            TrackedDeviceClass_Controller,
            g_left.get()
        );

        VRServerDriverHost()->TrackedDeviceAdded(
            "PhoneController_Right",
            TrackedDeviceClass_Controller,
            g_right.get()
        );

        // Startup UDP socket
        WSADATA wsa;
        int wsaErr = WSAStartup(MAKEWORD(2, 2), &wsa);
        if (wsaErr != 0) {
            DebugLog("WSAStartup failed: %d\n", wsaErr);
            return VRInitError_Driver_Failed;
        }

        g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (g_sock == INVALID_SOCKET) {
            DebugLog("Socket creation error: %d\n", WSAGetLastError());
            return VRInitError_Driver_Failed;
        }

        // Set socket to non-blocking mode
        u_long mode = 1;
        ioctlsocket(g_sock, FIONBIO, &mode);

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(5555);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        if (bind(g_sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            DebugLog("Socket bind error: %d\n", WSAGetLastError());
            closesocket(g_sock);
            g_sock = INVALID_SOCKET;
            return VRInitError_Driver_Failed;
        }

        DebugLog("UDP socket initialized on port 5555\n");
        return VRInitError_None;
    }

    void Cleanup() override {
        DebugLog("ServerDriver cleaning up...\n");
        if (g_sock != INVALID_SOCKET) {
            closesocket(g_sock);
            g_sock = INVALID_SOCKET;
        }
        WSACleanup();
    }

    const char* const* GetInterfaceVersions() override {
        return vr::k_InterfaceVersions;
    }

    void RunFrame() override {
        // Non-blocking recv
        char buf[4096];  // Increased buffer size for hand data
        sockaddr_in from;
        int fromlen = sizeof(from);
        int len = recvfrom(g_sock, buf, sizeof(buf) - 1, 0, (sockaddr*)&from, &fromlen);

        if (len > 0) {
            buf[len] = 0;
            try {
                auto j = json::parse(buf);

                // Process left hand
                if (j["left"]["pos"].is_array() && g_left) {
                    g_left->SetRawPose(
                        j["left"]["pos"].get<std::vector<float>>(),
                        j["left"]["rot"].get<std::vector<float>>()
                    );

                    // Process hand landmarks if available
                    if (j["left"]["landmarks"].is_array()) {
                        g_left->UpdateHandLandmarks(j["left"]["landmarks"].get<std::vector<std::vector<float>>>());
                    }
                }

                // Process right hand
                if (j["right"]["pos"].is_array() && g_right) {
                    g_right->SetRawPose(
                        j["right"]["pos"].get<std::vector<float>>(),
                        j["right"]["rot"].get<std::vector<float>>()
                    );

                    // Process hand landmarks if available
                    if (j["right"]["landmarks"].is_array()) {
                        g_right->UpdateHandLandmarks(j["right"]["landmarks"].get<std::vector<std::vector<float>>>());
                    }
                }
            }
            catch (const std::exception& e) {
                DebugLog("JSON parsing error: %s\n", e.what());
            }
            catch (...) {
                DebugLog("Unknown JSON parsing error\n");
            }
        }

        // Update poses
        if (g_left) g_left->UpdatePose();
        if (g_right) g_right->UpdatePose();
    }

    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}
};

// Initialize our global server driver instance
ServerDriver g_serverDriverInstance;

// Called once at startup
extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (!g_serverDriverHost) {
        g_serverDriverHost = &g_serverDriverInstance;
    }

    if (!_stricmp(pInterfaceName, IServerTrackedDeviceProvider_Version)) {
        return g_serverDriverHost;
    }

    if (pReturnCode)
        *pReturnCode = VRInitError_Init_InterfaceNotFound;
    return nullptr;
}
