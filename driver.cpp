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
#ifndef Prop_SupportsHandTracking_Bool
#define Prop_SupportsHandTracking_Bool static_cast<vr::ETrackedDeviceProperty>(3000) // Use a high unused ID
#endif
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

        PropertyContainerHandle_t container = VRProperties()->TrackedDeviceToPropertyContainer(_objectId);


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

        // Set controller input profile - Update the path prefix to match what SteamVR expects
        VRProperties()->SetStringProperty(
            _objectId,
            Prop_InputProfilePath_String,
            "{my_phonectrl}/input/handtracking_profile.json"
        );

        // Set controller type
        VRProperties()->SetStringProperty(
            _objectId,
            Prop_ControllerType_String,
            "my_phonectrl"  // Make sure this matches your input profile
        );
       
        // Register as supporting hand tracking
        VRProperties()->SetBoolProperty(
            _objectId,
            Prop_SupportsHandTracking_Bool,
            true
        );
        VRInputComponentHandle_t handle;

        VRDriverInput()->CreateSkeletonComponent(
            container,  // Use the property container from your device
            (_role == 0) ? "/input/skeleton/left" : "/input/skeleton/right",
            (_role == 0) ? "/skeleton/hand/left" : "/skeleton/hand/right",
            "/pose/raw",
            VRSkeletalTracking_Full,
            nullptr,
            0,
            &_skeletalComponentHandle
        );

        // Set hand tracking priority
        VRProperties()->SetInt32Property(
            _objectId,
            Prop_ControllerHandSelectionPriority_Int32,
            INT32_MAX
        );

        // Create skeletal components
        CreateSkeletalComponents();

        DebugLog("PhoneController %s activated with ID %u\n",
            _role == 0 ? "Left" : "Right", unObjectId);

        return VRInitError_None;
    }
    // Add this as a private method in the PhoneController class
    void CalculateBoneOrientations() {
        // Calculate orientations for each bone based on positions
        float wristToMiddle[3] = {
        _boneTransforms[9].position.v[0] - _boneTransforms[0].position.v[0],
        _boneTransforms[9].position.v[1] - _boneTransforms[0].position.v[1],
        _boneTransforms[9].position.v[2] - _boneTransforms[0].position.v[2]
        };

        float wristToIndex[3] = {
            _boneTransforms[5].position.v[0] - _boneTransforms[0].position.v[0],
            _boneTransforms[5].position.v[1] - _boneTransforms[0].position.v[1],
            _boneTransforms[5].position.v[2] - _boneTransforms[0].position.v[2]
        };

        // Now calculate orientation for each bone
        for (int i = 1; i < HAND_BONE_COUNT; i++) {
            int parent = GetParentBoneIndex(i);
            if (parent < 0) continue;

            // Get direction from parent to current bone
            float dir[3] = {
                _boneTransforms[i].position.v[0] - _boneTransforms[parent].position.v[0],
                _boneTransforms[i].position.v[1] - _boneTransforms[parent].position.v[1],
                _boneTransforms[i].position.v[2] - _boneTransforms[parent].position.v[2]
            };

            // Skip if direction is too small
            float length = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            if (length < 0.0001f) {
                // Default identity quaternion
                _boneTransforms[i].orientation.w = 1.0f;
                _boneTransforms[i].orientation.x = 0.0f;
                _boneTransforms[i].orientation.y = 0.0f;
                _boneTransforms[i].orientation.z = 0.0f;
                continue;
            }

            // Normalize direction
            dir[0] /= length;
            dir[1] /= length;
            dir[2] /= length;

            // Create a local coordinate system for the bone
            // Forward is the direction from parent to current
            float forward[3] = { dir[0], dir[1], dir[2] };

            // Find appropriate up vector based on bone type
            float up[3] = { 0.0f, 1.0f, 0.0f };  // default

            // For finger bones, we can use the wrist-to-middle direction as a reference
            if (i >= 5) {  // Finger bones
                up[0] = wristToMiddle[0];
                up[1] = wristToMiddle[1];
                up[2] = wristToMiddle[2];
            }

            // Make sure up is perpendicular to forward
            // First, calculate the dot product
            float dot = up[0] * forward[0] + up[1] * forward[1] + up[2] * forward[2];

            // Subtract the projection to make perpendicular
            up[0] -= dot * forward[0];
            up[1] -= dot * forward[1];
            up[2] -= dot * forward[2];

            // Normalize up
            length = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
            if (length < 0.0001f) {
                // If up vector is too small, find another vector perpendicular to forward
                if (fabs(forward[0]) < 0.9f) {
                    up[0] = 1.0f;
                    up[1] = 0.0f;
                    up[2] = 0.0f;
                }
                else {
                    up[0] = 0.0f;
                    up[1] = 1.0f;
                    up[2] = 0.0f;
                }

                // Make perpendicular to forward
                dot = up[0] * forward[0] + up[1] * forward[1] + up[2] * forward[2];
                up[0] -= dot * forward[0];
                up[1] -= dot * forward[1];
                up[2] -= dot * forward[2];

                // Normalize again
                length = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
            }

            up[0] /= length;
            up[1] /= length;
            up[2] /= length;

            // Calculate right as cross product of forward and up
            float right[3] = {
                forward[1] * up[2] - forward[2] * up[1],
                forward[2] * up[0] - forward[0] * up[2],
                forward[0] * up[1] - forward[1] * up[0]
            };

            // Now we have a local basis (right, up, forward)
            // Convert to quaternion - this is a standard way to convert a rotation matrix to quaternion

            // This is essentially constructing a 3x3 rotation matrix from the basis vectors
            // and then converting that matrix to a quaternion

            float trace = right[0] + up[1] + forward[2];
            float qw, qx, qy, qz;

            if (trace > 0.0f) {
                float s = 0.5f / sqrt(trace + 1.0f);
                qw = 0.25f / s;
                qx = (up[2] - forward[1]) * s;
                qy = (forward[0] - right[2]) * s;
                qz = (right[1] - up[0]) * s;
            }
            else if (right[0] > up[1] && right[0] > forward[2]) {
                float s = 2.0f * sqrt(1.0f + right[0] - up[1] - forward[2]);
                qw = (up[2] - forward[1]) / s;
                qx = 0.25f * s;
                qy = (right[1] + up[0]) / s;
                qz = (right[2] + forward[0]) / s;
            }
            else if (up[1] > forward[2]) {
                float s = 2.0f * sqrt(1.0f + up[1] - right[0] - forward[2]);
                qw = (forward[0] - right[2]) / s;
                qx = (right[1] + up[0]) / s;
                qy = 0.25f * s;
                qz = (up[2] + forward[1]) / s;
            }
            else {
                float s = 2.0f * sqrt(1.0f + forward[2] - right[0] - up[1]);
                qw = (right[1] - up[0]) / s;
                qx = (right[2] + forward[0]) / s;
                qy = (up[2] + forward[1]) / s;
                qz = 0.25f * s;
            }

            // Store the calculated quaternion
            _boneTransforms[i].orientation.w = qw;
            _boneTransforms[i].orientation.x = qx;
            _boneTransforms[i].orientation.y = qy;
            _boneTransforms[i].orientation.z = qz;

            // Normalize the quaternion to be safe
            float qLength = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            if (qLength > 0.0001f) {
                _boneTransforms[i].orientation.w /= qLength;
                _boneTransforms[i].orientation.x /= qLength;
                _boneTransforms[i].orientation.y /= qLength;
                _boneTransforms[i].orientation.z /= qLength;
            }
        }
    }
    void Deactivate() override {
        DebugLog("%s hand: Index MCP orientation: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
            _role == 0 ? "Left" : "Right",
            _boneTransforms[5].orientation.w,
            _boneTransforms[5].orientation.x,
            _boneTransforms[5].orientation.y,
            _boneTransforms[5].orientation.z);
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
            // Log position and orientation when updated
            DebugLog("%s hand pose: pos(%.3f, %.3f, %.3f) quat(%.3f, %.3f, %.3f, %.3f)\n",
                _role == 0 ? "Left" : "Right",
                p.vecPosition[0], p.vecPosition[1], p.vecPosition[2],
                p.qRotation.w, p.qRotation.x, p.qRotation.y, p.qRotation.z);

            VRServerDriverHost()->TrackedDevicePoseUpdated(_objectId, p, sizeof(p));
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
        if (landmarks.empty()) {
            DebugLog("%s hand: No landmarks received\n", _role == 0 ? "Left" : "Right");
            return;
        }
        DebugLog("%s hand: Received %d landmarks\n", _role == 0 ? "Left" : "Right", landmarks.size());
        // Log a few landmark positions
        if (landmarks.size() >= 21) {
            DebugLog("%s hand: Wrist at (%.3f, %.3f, %.3f)\n",
                _role == 0 ? "Left" : "Right",
                landmarks[0][0], landmarks[0][1], landmarks[0][2]);

            DebugLog("%s hand: Index tip at (%.3f, %.3f, %.3f)\n",
                _role == 0 ? "Left" : "Right",
                landmarks[8][0], landmarks[8][1], landmarks[8][2]);
        }
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
        if (_objectId != k_unTrackedDeviceIndexInvalid) {
            PropertyContainerHandle_t container = VRProperties()->TrackedDeviceToPropertyContainer(_objectId);

            // Create the skeletal component
            EVRInputError err = VRDriverInput()->CreateSkeletonComponent(
                container,
                (_role == 0) ? "/input/skeleton/left" : "/input/skeleton/right",
                (_role == 0) ? "/skeleton/hand/left" : "/skeleton/hand/right",
                "/pose/raw",
                VRSkeletalTracking_Full,
                nullptr,
                0,
                &_skeletalComponentHandle
            );
            // Add these lines to register the raw pose input source
            VRDriverInput()->CreateScalarComponent(
                container,
                "/pose/raw",
                &_poseComponentHandle,
                VRScalarType_Absolute,
                VRScalarUnits_None
            );

            DebugLog("%s hand: Created skeletal component, error=%d, handle=%llu\n",
                _role == 0 ? "Left" : "Right", err, _skeletalComponentHandle);

            // Now update both motion ranges for compatibility
            if (_skeletalComponentHandle != k_ulInvalidInputComponentHandle) {
                err = VRDriverInput()->UpdateSkeletonComponent(
                    _skeletalComponentHandle,
                    VRSkeletalMotionRange_WithController,
                    _boneTransforms.data(),
                    (uint32_t)_boneTransforms.size()
                );
                DebugLog("%s hand: Updated skeleton WithController, error=%d\n",
                    _role == 0 ? "Left" : "Right", err);

                err = VRDriverInput()->UpdateSkeletonComponent(
                    _skeletalComponentHandle,
                    VRSkeletalMotionRange_WithoutController,
                    _boneTransforms.data(),
                    (uint32_t)_boneTransforms.size()
                );
                DebugLog("%s hand: Updated skeleton WithoutController, error=%d\n",
                    _role == 0 ? "Left" : "Right", err);
            }
        }
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

        // SteamVR uses a different coordinate system than MediaPipe
            // MediaPipe: +Y is down, +Z is forward (toward camera)
            // SteamVR: +Y is up, +Z is backward

            // For all bones, apply the same coordinate conversion
            for (int i = 0; i < 21 && i < HAND_BONE_COUNT; i++) {
                if (landmarks[i].size() < 3) continue;

                // Set default orientation
                _boneTransforms[i].orientation.w = 1.0f;
                _boneTransforms[i].orientation.x = 0.0f;
                _boneTransforms[i].orientation.y = 0.0f;
                _boneTransforms[i].orientation.z = 0.0f;

                // First pass: Set all positions correctly
                for (int i = 0; i < 21 && i < HAND_BONE_COUNT; i++) {
                    if (landmarks[i].size() < 3) continue;

                    // Always be consistent with coordinate conversion
                    _boneTransforms[i].position.v[0] = landmarks[i][0] * scale;
                    _boneTransforms[i].position.v[1] = -landmarks[i][1] * scale;  // Flip Y
                    _boneTransforms[i].position.v[2] = -landmarks[i][2] * scale;  // Flip Z
                }
            }
            // Second pass: Calculate orientations
            CalculateBoneOrientations();
        // Process each bone
        for (int i = 1; i < 21 && i < HAND_BONE_COUNT; i++) {
            // Set position
            _boneTransforms[i].position.v[0] = landmarks[i][0] * scale;
            _boneTransforms[i].position.v[1] = landmarks[i][1] * scale;
            _boneTransforms[i].position.v[2] = landmarks[i][2] * scale;

            // Calculate bone orientation from parent to child
            int parent = GetParentBoneIndex(i);
            if (parent >= 0) {
                // Direction vector from parent to current bone
                float dx = _boneTransforms[i].position.v[0] - _boneTransforms[parent].position.v[0];
                float dy = _boneTransforms[i].position.v[1] - _boneTransforms[parent].position.v[1];
                float dz = _boneTransforms[i].position.v[2] - _boneTransforms[parent].position.v[2];

                // Normalize
                float len = sqrt(dx * dx + dy * dy + dz * dz);
                if (len > 0.0001f) {
                    dx /= len; dy /= len; dz /= len;

                    // Calculate rotation as quaternion from direction vector
                    // This creates a quaternion that rotates the default bone direction to the target direction

                    // Assuming default bone direction is (0,0,1) in SteamVR coordinates
                    // Which is actually (0,0,-1) after our coordinate flip
                    float angle = acos(-dz);  // Adjusted for flipped Z

                    if (angle > 0.0001f) {
                        float sinHalfAngle = sin(angle * 0.5f);
                        float cosHalfAngle = cos(angle * 0.5f);
                        // Cross product of (0,0,1) with direction gives rotation axis
                        float rx = -dy;  // Adjusted for flipped Z
                        float ry = dx;   // Adjusted for flipped Z
                        float rz = 0;

                        // Normalize rotation axis
                        float axisLen = sqrt(rx * rx + ry * ry);
                        if (axisLen > 0.0001f) {
                            rx /= axisLen;
                            ry /= axisLen;

                            _boneTransforms[i].orientation.w = cosHalfAngle;
                            _boneTransforms[i].orientation.x = rx * sinHalfAngle;
                            _boneTransforms[i].orientation.y = ry * sinHalfAngle;
                            _boneTransforms[i].orientation.z = rz * sinHalfAngle;
                        }
                    }
                }
            }
            // Log a few key transformations
            DebugLog("%s hand: Wrist at (%.3f, %.3f, %.3f) orientation (%.3f, %.3f, %.3f, %.3f)\n",
                _role == 0 ? "Left" : "Right",
                _boneTransforms[0].position.v[0],
                _boneTransforms[0].position.v[1],
                _boneTransforms[0].position.v[2],
                _boneTransforms[0].orientation.w,
                _boneTransforms[0].orientation.x,
                _boneTransforms[0].orientation.y,
                _boneTransforms[0].orientation.z);
        }
    }

    int GetParentBoneIndex(int boneIndex) {
        // This is a simplified mapping for a 31-bone hand skeleton
         // In a real implementation, you would map this accurately to SteamVR's bone hierarchy
         // The typical SteamVR hand has 31 bones:
         // - 1 root/wrist
         // - 5 metacarpals (1 per finger)
         // - 5 knuckles (1 per finger)
         // - 4 finger proximal joints (excluding thumb)
         // - 4 finger middle joints (excluding thumb)
         // - 5 finger tip joints (1 per digit)
         // - 6 auxiliary/helper bones

         // For this example, we'll map directly from MediaPipe's 21 landmarks:
         // 0: Wrist
         // 1-4: Thumb (CMC, MCP, IP, TIP)
         // 5-8: Index (MCP, PIP, DIP, TIP)
         // 9-12: Middle (MCP, PIP, DIP, TIP)
         // 13-16: Ring (MCP, PIP, DIP, TIP)
         // 17-20: Pinky (MCP, PIP, DIP, TIP)

        if (boneIndex == 0) return -1;  // Wrist is root

        // Thumb chain
        if (boneIndex >= 1 && boneIndex <= 4) {
            return boneIndex - 1;  // Each thumb bone connects to previous
        }

        // For fingers (index, middle, ring, pinky)
        // The MCP (knuckle) connects to wrist
        if (boneIndex == 5 || boneIndex == 9 || boneIndex == 13 || boneIndex == 17) {
            return 0;  // Connect to wrist
        }

        // Otherwise, connect to previous bone in the chain
        return boneIndex - 1;
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
        static int frameCount = 0;
        if (++frameCount % 100 == 0) {  // Log every 100 frames
            DebugLog("Driver running frame %d\n", frameCount);
        }
        // Non-blocking recv
        char buf[8192];  // Larger buffer size for hand data
        sockaddr_in from;
        int fromlen = sizeof(from);
        int len = recvfrom(g_sock, buf, sizeof(buf) - 1, 0, (sockaddr*)&from, &fromlen);

        if (len > 0) {
            buf[len] = 0;

            // Don't log the entire buffer, it's too large
            DebugLog("Received UDP data: %d bytes\n", len);

            try {
                auto j = json::parse(buf);
                // In the C++ driver:
                DebugLog("Received valid hand data for %s hand with %d landmarks\n",
                    j["left"].contains("landmarks") ? "left" : "right",
                    j["left"].contains("landmarks") ? j["left"]["landmarks"].size() : 0);
                // Process left hand
                if (j.contains("left") && j["left"].contains("pos") &&
                    j["left"]["pos"].is_array() && g_left) {

                    std::vector<float> leftPos = j["left"]["pos"].get<std::vector<float>>();
                    std::vector<float> leftRot = j["left"]["rot"].get<std::vector<float>>();

                    if (leftPos.size() == 3 && leftRot.size() == 4) {
                        DebugLog("Left hand position: %.3f, %.3f, %.3f\n",
                            leftPos[0], leftPos[1], leftPos[2]);

                        g_left->SetRawPose(leftPos, leftRot);

                        // Process hand landmarks if available
                        if (j["left"].contains("landmarks") && j["left"]["landmarks"].is_array()) {
                            auto landmarks = j["left"]["landmarks"].get<std::vector<std::vector<float>>>();
                            DebugLog("Left hand landmarks count: %d\n", landmarks.size());
                            g_left->UpdateHandLandmarks(landmarks);
                        }
                        else {
                            DebugLog("No left hand landmarks in data\n");
                        }
                    }
                }

                // Process right hand
                if (j.contains("right") && j["right"].contains("pos") &&
                    j["right"]["pos"].is_array() && g_right) {

                    std::vector<float> rightPos = j["right"]["pos"].get<std::vector<float>>();
                    std::vector<float> rightRot = j["right"]["rot"].get<std::vector<float>>();

                    if (rightPos.size() == 3 && rightRot.size() == 4) {
                        DebugLog("Right hand position: %.3f, %.3f, %.3f\n",
                            rightPos[0], rightPos[1], rightPos[2]);

                        g_right->SetRawPose(rightPos, rightRot);

                        // Process hand landmarks if available
                        if (j["right"].contains("landmarks") && j["right"]["landmarks"].is_array()) {
                            auto landmarks = j["right"]["landmarks"].get<std::vector<std::vector<float>>>();
                            DebugLog("Right hand landmarks count: %d\n", landmarks.size());
                            g_right->UpdateHandLandmarks(landmarks);
                        }
                        else {
                            DebugLog("No right hand landmarks in data\n");
                        }
                    }
                }
            }
            catch (const json::exception& e) {
                DebugLog("JSON parsing error: %s\n", e.what());
                // Log the first 100 characters of the buffer for debugging
                char logBuffer[101];
                strncpy_s(logBuffer, buf, 100);
                logBuffer[100] = '\0';
                DebugLog("Buffer start: %s\n", logBuffer);
            }
            catch (const std::exception& e) {
                DebugLog("Standard exception: %s\n", e.what());
            }
            catch (...) {
                DebugLog("Unknown error while processing UDP data\n");
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
